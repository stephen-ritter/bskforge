# bskforge — Implementation Roadmap

> Living document. Update **In progress** at every work step, move finished items
> to **Completed** (dated, with the Basilisk version validated against), and keep
> **Up next** ordered by priority. Definition of done per item: `ruff check`
> clean, `ty check` clean, tests passing.

Target Basilisk: **2.12.x (PyPI `bsk[all]`)**, authored against the 2.13.0b0
checkout in `../basilisk` for behavior/reference.

---

## In progress

Phase 6 — fleets & operations: comm links, formation flying wiring,
Monte Carlo dispersions. Design pass on the Basilisk comm/formation
modules in flight.

---

## Up next

### Phase 2 — Environment fidelity  ✅ (see Completed)
- [x] SPICE/Pooch epoch + kernels via `DataFile` enums (`createSpiceInterface`)
- [x] Spherical-harmonics gravity (`useSphericalHarmonicsGravityModel`);
      polyhedral bodies remain available through `[custom]`
- [x] Magnetic field: WMM, centered dipole (`simSetPlanetEnvironment`) +
      magnetometer sensor (`magInMsg` wiring)
- [x] Eclipse (penumbra/umbra), CSS + SRP shadow feed
- [x] Atmosphere: exponential, MSIS (static space-weather inputs); 2.12
      ships only a zero wind model, so wind stays `[custom]`-only
- [x] SRP (cannonball + faceted), drag (cannonball + faceted) as disturbances
- [x] Global and per-vehicle environment override wiring
- [x] Disturbance effectors: gravity gradient, drag, SRP (third-body gravity
      is automatic: any SPICE `[[world.gravities]] body` perturbs vehicles)
- [ ] Solar-flux override: no consumer module in the 2.12 wheel; remains an
      explicit `UnsupportedError`

### Phase 3 — Sensors & estimation  ✅ (see Completed)

### Phase 4 — Effector breadth  ✅ (see Completed)
- [x] Thrusters (`simIncludeThruster`) + `thrForceMapping`, firing (Schmitt/Round),
      `thrMomentumDumping`
- [x] MTB + momentum management (`mtbMomentumManagementSimple`, `dipoleMapping`)
- [x] RW null-space desaturation (`rwNullSpace`); `thrustRWDesat` is superseded
      by `thrMomentumDumping` upstream and stays `[custom]`-only
- [ ] VSCMG (stretch; `thrusterStateEffector` + CMG effector via `[custom]`)

### Phase 5 — FSW breadth  ✅ (see Completed)
- [x] Guidance set: `velocityPoint`, `hillPoint`, `sunSafePoint`, `waypointReference`,
      `locationPointing`, `celestialTwoBodyPoint`
- [x] Control: `mrpPD`, `rateServoFullNonlinear`, `mrpSteering`

### Phase 6 — Fleets & operations
- [ ] Comm links (`simpleAntenna`, `linkBudget`)
- [ ] Formation FSW modules and formation flying wiring
      (`formationBarycenter`, `hillStateConverter`)
- [ ] Monte Carlo (`MonteCarlo.Controller`) + dispersions from TOML
- [ ] Hook-based event effects beyond `[custom.events]` (already available:
      time-conditioned events + `pkg.module:function` hooks)
- [ ] MuJoCo scenes (stretch)

### Phase 7 — Polish & docs
- [ ] Generated TOML reference from the pydantic JSON schema
- [ ] Expanded example library mirroring canonical Basilisk scenarios
- [ ] `validate`/`graph` UX polish; error-message quality pass

---

## Completed

### 2026-11-20 — Phase 5: guidance laws and control laws
Validated end-to-end against installed **Basilisk 2.12.2** (new example
`examples/guidance_laws.toml` + 11 new tests, 73 total passing).
- [x] Guidance union under `[spacecraft.fsw.guidance]`: `velocityPoint`
      (auto `mu` from the world's central body), `hillPoint`, `sunSafePoint`
      (AttGuidMsg straight into the control law, fed by the
      `vehSunPntBdy` field), `locationPointing` (one of `target_body`,
      `target_latlon`, `target_position_N`), `celestialTwoBodyPoint`
      (`target_body` / `constraint_body`) and `waypointReference`
      (file paths resolved against the scenario file).
- [x] `_SpiceToEphemeris` dyn-task bridge converts `SpicePlanetStateMsg`
      to `EphemerisMsg` for `celBodyInMsg`/`sunPositionInMsg` ports;
      `_GroundStateWriter` publishes geodetic sites as `GroundStateMsg`
      (ellipsoid normal + `J20002Pfix` rotation). Both feed only from
      the SPICE ephemeris hub; celestial-referencing guidance requires
      `[world.spice]` (validated at the root).
- [x] `_ensure_trans_msg` reuses the estimator's `transOutMsg` or
      inserts a silent `SimpleNav`, so guidance laws get `scTransInMsg`
      without schema ceremony.
- [x] Control union extended with `mrpPD` and `rateServo`
      (`mrpSteering` → `rateServoFullNonlinear`, steering gains
      `steering_k1`/`steering_k3`/`max_rate`, wheel parameters and
      speeds auto-linked when an RW set exists).
- [x] **Process priorities fixed**: Basilisk steps processes in
      *descending* priority, so the dynamics process now runs at 3, the
      FSW process at 2 and custom processes (default 1) last. With the
      old inverted priorities, SPICE ephemerides were one FSW tick
      stale, which NaNed `celestialTwoBodyPoint` at its first
      uninitialized read; the `nav_state` recorder now samples on the
      FSW task so estimator frames are time-aligned.

### 2026-11-20 — Phase 4: thrusters, torque bars, momentum management
Validated end-to-end against installed **Basilisk 2.12.2** (new example
`examples/momentum_management.toml` + 10 new tests, 61 total passing).
- [x] Thruster sets (`[[spacecraft.effectors]] type = "thrusters"`):
      per-device `direction_B` / `position_B` / `max_thrust` / `divergence`
      through `simIncludeThruster.thrusterFactory` +
      `thrusterDynamicEffector` (force **and** torque on the hub; the
      factory's `create()` kwargs pass through).
- [x] `thrusterTorque` allocation: `thrForceMapping` (validated geometry —
      `thrForceSign = +1`, body-axis `controlAxes_B`) feeding the firing
      logic, `thrFiringSchmitt` (default) or `thrFiringRound`, whose
      on-times actuate the effector through `THRArrayOnTimeCmdMsg`.
      Pure-RCS attitude control converges with real pulses.
- [x] MTB sets (`type = "mtb"`): `MtbEffector` + component-major
      `MTBArrayConfigMsg` (2.9.2+ axis-major `GtMatrix_B` layout); the
      effector subscribes itself to the `[environment.magnetic_field]`
      message and the build fails with a clear `BuildError` when none is
      configured (an MTB set in vacuum is torque-free).
- [x] `dipoleMapping` allocation: `torque2Dipole` + `dipoleMapping`
      steering (auto-computed Moore–Penrose pseudo-inverse of the bar
      axes, overridable via `steering_matrix`). `tamComm` needs a written
      `dcm_BS` (its default zero matrix trips a Basilisk bug check) —
      wired to identity.
- [x] Momentum management (`[spacecraft.fsw.momentum_management]`):
      `thrMomentumDumping` (`thrMomentumManagement` → `thrForceMapping` →
      `thrMomentumDumping`; wheel momentum 92 → 9 N·m·s while pointing
      holds) and `mtbMomentumManagementSimple` (magnetic desat: 0.92 →
      0.37 N·m·s with saturated 50 A·m² bars).
      `thrMomentumManagement` computes delta-H once per `Reset` and
      `thrMomentumDumping` ignores t=0-stamped requests, so a small Python
      bridge re-arms the evaluation every `rearm_period` seconds.
- [x] Null-space desaturation (`[spacecraft.fsw.desaturation]`):
      `rwNullSpace` steers four-wheel pyramids toward desired speeds
      without fighting the attitude loop (rewires `rwMotorTorque` to the
      effector). `thrustRWDesat` is superseded upstream; stays `[custom]`.
- [x] Allocation/effector cross-checks: every allocation and management
      scheme names its required effector kind, validated at load
      (`spacecraft.effectors` must contain it); two writers of one
      effector command input (e.g. `thrusterTorque` + thruster dumping)
      raise a `BuildError` naming both.
- [x] Control law → effector exclusivity: with an allocation configured the
      commanded torque reaches the hub *only* through the effectors
      (`extFT` stays unlinked); previously the control output also torqued
      the hub directly, inflating authority for wheel/thruster vehicles.
      RW factory now honours `useMaxTorque` so `max_torque` limits bite.
- [x] New auto-recorded streams `{sc}.thruster_cmds` (`OnTimeRequest`,
      commanded forces) and `{sc}.mtb_cmds` (`mtbDipoleCmds`).

### 2026-11-20 — Phase 3: sensors & estimation
Validated end-to-end against installed **Basilisk 2.12.2** (new example
`examples/estimation_and_opnav.toml` + 9 new tests, 51 total passing).
- [x] Pinhole landmark camera (`pinholeCamera`): `focal_length` /
      `pixel_size` / `resolution` optics knobs (Basilisk's `Reset()` derives
      the FOV), `sigma_CB` mounting, `landmarks` (position + normal in the
      target body-fixed frame), `sun_mask_angle`; per-landmark output
      messages recorded (`{cam}`, `{cam}_2`, … with `pL`, `isVisible`, …).
      Lighting uses the target's origin-relative position, so the camera
      requires SPICE ephemerides **and** a sun-centered inertial frame:
      new `[world.spice] zero_base = "sun"` knob sets `SpiceInterface.zeroBase`.
- [x] CSS constellations: `[[spacecraft.css_constellations]]` groups named
      CSS units into a `CSSConstellation` with a written `CSSConfigMsg`
      (`CBias = 1.0` — the config default of 0 would zero measurements);
      unknown members rejected at validation, unknown clusters at build with
      the available-cluster list.
- [x] Estimators (`[spacecraft.fsw.estimator]`): `cssWlsEst` (`use_weights`),
      `sunlineSuKF` (`q_obs`, `output_sunline`) and `inertialUKF` — ST-fused
      sigma-point filter with sane defaults (module defaults of 0 give NaN;
      α/β/κ/covar/stateInit presets, per-ST measurement-noise diagonal).
      The ST→UKF gap (STSensorMsg quaternion vs STAttMsg MRP) is bridged by
      a small Python `SysModel` (`EP2MRP`, skips not-yet-written sensors);
      the module's mandatory RW/gyro ports subscribe to written-zero
      dummy messages when no wheels are configured.
- [x] Estimator output auto-recorded as `{sc}.nav_state` (MRP, rate, sun
      pointer per estimator outputs).
- [x] Sensor noise polish: ST `white_noise` (seeded RNG — `RNGSeed = crc32`
      of the model tag, since Basilisk refuses noise without a non-zero
      seed), IMU `gyro_bias`/`gyro_walk`/`accel_walk`.
- [x] Physics tests: cssWlsEst sunline = truth sun direction (unit norm),
      inertialUKF converges to truth attitude (< 1e−3), landmark camera
      observes a surface point (pixels inside the array), ST noise scatters
      around truth, IMU bias appears in gyro output, cluster/SPICE guards.
- Notes: un-torqued-limited RW loops diverge — set per-wheel
  `max_torque`; the `camera` type stays guarded when Basilisk lacks the
  opnav build feature (`hasBuildFeature("opNav")` guards future opnav
  modules; pinhole itself needs no feature).

### 2026-11-20 — Phase 2: environment fidelity
Validated end-to-end against installed **Basilisk 2.12.2** (new example
`examples/earth_environment.toml` + 12 new tests, 40 total passing).
- [x] `[world.spice]`: epoch-driven `createSpiceInterface`, kernel selection
      by `DataFile.EphemerisData` name, automatic `loadEphemeris` for
      non-central bodies, `zeroBase` = central body; per-body
      `SpicePlanetStateMsg` registry (`ctx.planet_state_msgs`). The sun
      message feeding CSS/SRP/eclipse becomes live SPICE when the sun is a
      listed gravity body.
- [x] Spherical-harmonics gravity: `grav_order` + optional `grav_file`
      (per-planet canonical defaults, e.g. Earth → GGM03S).
- [x] Third-body gravity is implicit: extra `[[world.gravities]]` bodies with
      SPICE ephemerides perturb every vehicle (moon in the example).
- [x] `[environment]` models built and shared per configuration:
      magnetic field (centered dipole presets / WMM file with `[world].epoch`
      message), atmosphere (exponential presets / MSIS with static
      Ap/F10.7), penumbra eclipse (sun + central-body messages wired,
      `rEqCustom` from the body radius).
- [x] Per-vehicle `[spacecraft.environment]` deep-merged over the global
      section (`resolve_environment`); environment modules attach any number
      of vehicles (`addSpacecraftToModel` index → per-vehicle output msg).
- [x] `[[spacecraft.disturbances]]`: gravity gradient, drag (cannonball /
      faceted plates), SRP (cannonball / faceted with diffuse+specular
      coefficients), each fed from the environment messages and attached as
      dynamic effectors.
- [x] Magnetometer sensor built for real (`magInMsg` ← field module,
      `dcm_SB` from `sigma_SB`, noise σ/bias/saturation; `tamDataOutMsg`
      recorded).
- [x] Phase order updated: world → spacecraft → environment → disturbances →
      effectors → sensors → FSW → custom → recorders → events → viz.
- [x] Physics-level tests: WMM field magnitude at LEO, SRP cannonball force
      vs analytic expectation, drag density chain, gravity-gradient torque,
      SPICE sun distance ≈ 1 au, TAM vector rotation over an orbit, faceted
      facet counts + forces.
- Notes: `forceExternal_B` vs `_N` retention differs between
  `dragDynamicEffector` (B kept) and `radiationPressure` (N kept);
  `FacetSRPDynamicEffector` requires `setNumFacets()` before `addFacet()`.

### 2026-11-20 — Phase 1: core rewrite (schema v2 + builders + `[custom]` escape hatch)
Validated end-to-end against installed **Basilisk 2.12.2** (`bsk[all]` wheel).
- [x] Package layout: `schema/`, `loader.py`, `build/` (phase registry),
      `export.py`, `runner.py`, `cli.py` (the `resolve/` duties live in
      `loader.py`: `extends` deep-merge + circular/missing-parent errors).
- [x] `schema/` — strict pydantic v2 models, `type = "..."` discriminated
      unions, `extra="forbid"` with did-you-mean suggestions resolved against
      the exact document position (walks the model tree).
- [x] Thin-semantics design: blessed knobs + free-form `params` passthrough
      checked against the installed module via introspection (`apply_params`).
- [x] Orbit `elements`/`rv`, attitude init (MRP + rate), hub inertia as
      symmetric table `{ixx..iyz}` or flat 9-vector.
- [x] World: `gravBodyFactory` presets + `createCustomGravObject`, central
      body conflict validation; SPICE toggle fails with a Phase-2 pointer.
- [x] `SimContext`: dynamics (priority 1) / FSW (priority 2) processes and
      tasks, tag registry for message cross-references, task/process
      resolution for `[custom]`.
- [x] Builders: spacecraft hub + `extForceTorque`, RW sets via `rwFactory`
      (2.12 API: `Js`, RPM `Omega`, `rwSpeedOutMsg`, jitter enums), CSS/ST/IMU
      sensors (2.11 Eigen conventions via `simHelpers.np2EigenMatrix3d`),
      FSW graph `simpleNav → attTrackingError → inertial3D → mrpFeedback →
      rwMotorTorque` (incl. `controlAxes_B`), recorders (auto sample time via
      `simHelpers.samplingTime`), Vizard attach via 2.8+ kwargs.
- [x] Multi-spacecraft: per-vehicle logging/FSW/effectors; duplicate-name and
      allocation-without-wheels cross-validation.
- [x] `[custom]` Layer-3 escape hatch (**pulled forward into Phase 1**):
      arbitrary Basilisk modules by import path, custom tasks/processes,
      `tag.message` connections, events (time condition or import-string
      hooks), `simulation_options` — makes the format able to express
      everything the installed Basilisk can do; demonstrated by
      `examples/custom_escape_hatch.toml` (torque low-pass re-wiring).
- [x] Export: recorder field-vector discovery → pandas → CSV/parquet/npz;
      `ScenarioRunResult.frames` + `to_directory`.
- [x] CLI: `bskforge run|validate|schema|graph` (graph = Basilisk
      `GetMessageConnectionDot`).
- [x] Examples rewritten to schema v2 (+ new `custom_escape_hatch.toml`);
      all four run end-to-end with logged output.
- [x] Tests: 28 passing (loader/schema/extends unit + build/run/export
      integration incl. closed-loop convergence check σ→1e-6, RW-controlled);
      ruff check/format clean, `ty` clean.

Notes / known behavior:
- Recorder samples at `t = 0` read messages before writers first execute —
  Basilisk emits one "not properly initialized" warning per stream and logs a
  zero first sample. This is stock Basilisk behavior (matches official
  examples); later samples are full-rate.
- Magnetometer and camera sensors raise `UnsupportedError` (need Phase-2
  magnetic-field wiring / Phase-3 imaging stack); `[world.spice]` and
  `[environment]` blocks validate but fail the build with a phase pointer.

### 2026-11-19 — Phase 0: rename, repo hygiene, environment, tooling
- Renamed `bsk_toml_sim` → `bskforge`; `git init` with legacy snapshot committed.
- Removed stale artifacts: empty `.venv`, `.mypy_cache` trees, `.egg-info`.
- Rewrote `pyproject.toml`: `bskforge` distribution + `bskforge` CLI entry point,
  `bsk[all]>=2.12.0` + `pydantic>=2.7` deps, `dev` extras (`pytest`, `pytest-cov`,
  `ruff`, `ty`).
- Added `[tool.ruff]` (E/W/F/I/UP/B/SIM/C4/RUF/ANN/D, google docstrings, format
  with docstring code formatting) and `[tool.ty]` (strict, `import-not-found=warn`
  for untyped Basilisk stubs).
- Added `.gitignore`, `README.md`, this `ROADMAP.md`.
- Environment: fresh venv, `pip install "bsk[all]"`; validated import + `printBuildInfo()`.
- Deleted the legacy `bsk_toml_sim` package and its mirror-schema tests.

---

## Basilisk module-family coverage checklist

Legend: ❌ not covered · 🟡 schema only · 🔧 partial · ✅ end-to-end

| Family | Module examples | Status |
| --- | --- | --- |
| Dynamics hub | `spacecraft` | ✅ |
| Gravity/world | `simIncludeGravBody`, SPICE, sph-harm, third-body | ✅ (polyhedral via `[custom]`) |
| Reaction wheels | `reactionWheelStateEffector`, `simIncludeRW` | ✅ |
| Thrusters | `thrusterStateEffector`, `thrusterDynamicEffector` | ✅ (state-model thrusters via `[custom]`) |
| MTB | `MtbEffector` | ✅ |
| SRP | `radiationPressure`, `facetSRPDynamicEffector` | ✅ |
| Drag | `dragDynamicEffector`, `facetDragDynamicEffector` | ✅ |
| Magnetic field | `magneticFieldWMM`, `magneticFieldCenteredDipole` | ✅ |
| Atmosphere | `ExponentialAtmosphere`, `MsisAtmosphere` | ✅ (2.12 has no wind model) |
| Eclipse | `eclipse` | ✅ |
| Sensors (att) | `coarseSunSensor`, `starTracker`, `imuSensor`, `magnetometer`, `pinholeCamera` | ✅ |
| Estimation | `simpleNav`, `InertialUKF`, `cssWlsEst`, `sunlineSuKF` | ✅ |
| Att guidance | `inertial3D` · `velocityPoint` · `hillPoint` · `sunSafePoint` · `locationPointing` · `celestialTwoBodyPoint` · `waypointReference` ✅ | ✅ |
| Att control | `mrpFeedback` · `mrpPD` · `rateServo` (`mrpSteering` + `rateServoFullNonlinear`) ✅ | ✅ |
| Att effector iface | `rwMotorTorque`, `thrForceMapping`, `dipoleMapping` | ✅ |
| Momentum mgmt | `thrMomentumManagement`, `thrMomentumDumping`, `mtbMomentumManagementSimple`, `rwNullSpace` | ✅ |
| Formation | `formationBarycenter`, `hillStateConverter` | ❌ (Phase 6; `[custom]` reachable) |
| Comms | `simpleAntenna`, `linkBudget` | ❌ (Phase 6; `[custom]` reachable) |
| Monte Carlo | `MonteCarlo.Controller` | ❌ (Phase 6) |
| Visualization | `vizSupport` | 🔧 (attach path built; headless capture test pending) |
| Data export | recorder field discovery → CSV/parquet/npz | ✅ |
| Escape hatch | `[custom]` modules/tasks/connections/events | ✅ |
