# bskforge — Implementation Roadmap

> Living document. Update **In progress** at every work step, move finished items
> to **Completed** (dated, with the Basilisk version validated against), and keep
> **Up next** ordered by priority. Definition of done per item: `ruff check`
> clean, `ty check` clean, tests passing.

Target Basilisk: **2.12.x (PyPI `bsk[all]`)**, authored against the 2.13.0b0
checkout in `../basilisk` for behavior/reference.

---

## In progress

_(nothing in flight — Phase 1 complete, see below)_

---

## Up next

### Phase 2 — Environment fidelity  _(user-prioritized next)_
- [ ] SPICE/Pooch epoch + kernels via `DataFile` enums (`createSpiceInterface`),
      replacing the static `[world.sun]` message for CSS/eclipse consumers
- [ ] Spherical-harmonics gravity (`useSphericalHarmonicsGravityModel` +
      `loadGravFromFile`), polyhedral bodies
- [ ] Magnetic field: WMM, centered dipole (`simSetPlanetEnvironment`) +
      magnetometer sensor builder (`magInMsg` wiring)
- [ ] Eclipse (umbra/penumbra), solar flux
- [ ] Atmosphere: MSIS, exponential, tabular; wind models
- [ ] SRP (plain + faceted), drag (plain + faceted) as effectors
- [ ] Global and per-vehicle environment override wiring (replace today's
      `UnsupportedError` guard)
- [ ] Disturbance effectors (gravity gradient, solar radiation pressure torque,
      third-body)

### Phase 3 — Sensors & estimation
- [ ] Sensor noise polish (`setAMatrix`, ST sampling period, IMU drifting bias)
- [ ] Sensor interfaces (`CSSSensorData`, `STSensorData`, `IMUSensorData`,
      `TAMSensorData`)
- [ ] Estimators: `InertialUKF`, `CSSEst`, `sunlineSuKF`
- [ ] Camera (+opnav guard via `Basilisk.hasBuildFeature`)

### Phase 4 — Effector breadth
- [ ] Thrusters (`simIncludeThruster`) + `thrForceMapping`, firing (Schmitt/Round),
      `thrMomentumDumping`
- [ ] MTB + momentum management (`mtbMomentumManagement*`, `dipoleMapping`)
- [ ] RW null-space, desaturation (`rwNullSpace`, `thrustRWDesat`)
- [ ] VSCMG (stretch)

### Phase 5 — FSW breadth
- [ ] Guidance set: `velocityPoint`, `hillPoint`, `sunSafePoint`, `waypointReference`,
      `locationPointing`, `celestialTwoBodyPoint`
- [ ] Control: `mrpPD`, `rateServoFullNonlinear`, `mrpSteering`
- [ ] Formation FSW modules; opnav stack (`bsk-opnav`)

### Phase 6 — Fleets & operations
- [ ] Comm links (`simpleAntenna`, `linkBudget`)
- [ ] Formation flying wiring (`formationBarycenter`, `hillStateConverter`)
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
| Gravity/world | `simIncludeGravBody` (presets, custom) | 🔧 (SPICE: Phase 2) |
| Reaction wheels | `reactionWheelStateEffector`, `simIncludeRW` | ✅ |
| Thrusters | `thrusterStateEffector` | ❌ (Phase 4; `[custom]` reachable) |
| MTB | `MtbEffector` | ❌ (Phase 4; `[custom]` reachable) |
| SRP | `facetSRPDynamicEffector`, `radiationPressure` | ❌ (Phase 2; `[custom]` reachable) |
| Drag | `dragDynamicEffector`, `facetDragDynamicEffector` | ❌ (Phase 2; `[custom]` reachable) |
| Magnetic field | `magneticFieldWMM`, `magneticFieldCenteredDipole` | 🟡 (Phase 2) |
| Atmosphere | `MsisAtmosphere`, `ExponentialAtmosphere` | 🟡 (Phase 2) |
| Eclipse | `eclipse` | 🟡 (Phase 2) |
| Sensors (att) | `coarseSunSensor`, `starTracker`, `imuSensor` ✅ · `magnetometer` ❌ | 🔧 |
| Estimation | `simpleNav` ✅ · `InertialUKF`, `CSSEst` ❌ | 🔧 |
| Att guidance | `inertial3D` ✅ · `velocityPoint`, `hillPoint` ❌ | 🔧 |
| Att control | `mrpFeedback` ✅ · `mrpPD`, steering laws ❌ | 🔧 |
| Att effector iface | `rwMotorTorque` ✅ · `thrForceMapping` ❌ | 🔧 |
| Formation | `formationBarycenter`, `hillStateConverter` | ❌ (Phase 6; `[custom]` reachable) |
| Comms | `simpleAntenna`, `linkBudget` | ❌ (Phase 6; `[custom]` reachable) |
| Monte Carlo | `MonteCarlo.Controller` | ❌ (Phase 6) |
| Visualization | `vizSupport` | 🔧 (attach path built; headless capture test pending) |
| Data export | recorder field discovery → CSV/parquet/npz | ✅ |
| Escape hatch | `[custom]` modules/tasks/connections/events | ✅ |
