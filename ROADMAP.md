# bskforge — Implementation Roadmap

> Living document. Update **In progress** at every work step, move finished items
> to **Completed** (dated, with the Basilisk version validated against), and keep
> **Up next** ordered by priority. Definition of done per item: `ruff check`
> clean, `ty check` clean, tests passing.

Target Basilisk: **2.12.x (PyPI `bsk[all]`)**, authored against the 2.13.0b0
checkout in `../basilisk` for behavior/reference.

---

## In progress

_(nothing in flight — Phase 0 complete, see below)_

---

## Up next

### Phase 1 — Core rewrite (schema + builder registry)
- [ ] Package layout: `schema/`, `resolve/`, `build/`, `export/`, `cli/`, `runner`
- [ ] `schema/` — strict pydantic v2 models; `type = "..."` discriminated unions;
      `extra="forbid"` with did-you-mean suggestions
- [ ] `schema/orbit` — `elements` (Keplerian), `rv`
- [ ] `schema/spacecraft` — hub, attitude, sensors, effectors, disturbances, fsw
- [ ] `schema/world` — gravity bodies (point mass), epoch, SPICE toggle
- [ ] `schema/environment` — global defaults, per-vehicle override
- [ ] `resolve/` — `extends` inheritance + default merge (global → per-vehicle)
- [ ] `build/context.py` — `SimContext` (processes/tasks, gravFactory, message registry)
- [ ] `build/` registry — phases: world → spacecraft → effectors → environment →
      sensors → fsw → logging → viz
- [ ] ReactionWheel effector fixes: `rwSpeedOutMsg`, RPM `Omega_max`, device
      registration before `InitializeSimulation()`
- [ ] Separate dynamics / FSW processes and tasks (correct priorities)
- [ ] Basilisk 2.8+ viz API (`enableUnityVisualization` kwargs, per-sc settings)
- [ ] Real `SimulationResult`: recorders → pandas → CSV/parquet; scalar traces
- [ ] `cli/` — `run`, `validate`, `schema`, `graph`
- [ ] Migrate the 3 example TOMLs; all three run end-to-end with logged output

### Phase 2 — Environment fidelity  _(user-prioritized next)_
- [ ] SPICE/Pooch epoch + kernels via `DataFile` enums (`createSpiceInterface`)
- [ ] Spherical-harmonics gravity (`useSphericalHarmonicsGravityModel` +
      `loadGravFromFile`), polyhedral, custom bodies (`createCustomGravObject`)
- [ ] Magnetic field: WMM, centered dipole (`simSetPlanetEnvironment`)
- [ ] Eclipse (umbra/penumbra), solar flux
- [ ] Atmosphere: MSIS, exponential, tabular; wind models
- [ ] SRP (plain + faceted), drag (plain + faceted) as effectors
- [ ] Global and per-vehicle environment override wiring

### Phase 3 — Sensors & estimation
- [ ] CSS, star tracker, magnetometer, IMU (noise via `senNoiseStd`/`setAMatrix`)
- [ ] Sensor interfaces (`CSSSensorData`, `STSensorData`, `IMUSensorData`, `TAMSensorData`)
- [ ] Estimators: `simpleNav`, `InertialUKF`, `CSSEst`, `sunlineSuKF`
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
- [ ] Events (`createNewEvent`) and time-tagged scenarios
- [ ] MuJoCo scenes (stretch)

### Phase 7 — Polish & docs
- [ ] Generated TOML reference from the pydantic JSON schema
- [ ] Expanded example library mirroring canonical Basilisk scenarios
- [ ] `validate`/`graph` UX polish; error-message quality pass

---

## Completed

### 2026-11-19 — Phase 0: rename, repo hygiene, environment, tooling
- Renamed `bsk_toml_sim` → `bskforge`; `git init` with legacy snapshot committed.
- Removed stale artifacts: empty `.venv`, `.mypy_cache` trees, `.egg-info`.
- Rewrote `pyproject.toml`: `bskforge` distribution + `bskforge` CLI entry point,
  `bsk[all]>=2.12.0` + `pydantic>=2.7` deps, `dev` extras (`pytest`, `pytest-cov`,
  `ruff`, `ty`).
- Added `[tool.ruff]` (E/W/F/I/UP/B/SIM/C4/RUF/ANN/D, google docstrings, format
  with docstring code formatting) and `[tool.ty]` (strict, `unresolved-import=warn`
  for untyped Basilisk stubs).
- Added `.gitignore`, `README.md`, this `ROADMAP.md`.
- Environment: fresh venv, `pip install "bsk[all]"`; validated import + `printBuildInfo()`.

---

## Basilisk module-family coverage checklist

Legend: ❌ not covered · 🟡 schema only · 🔧 partial · ✅ end-to-end

| Family | Module examples | Status |
| --- | --- | --- |
| Dynamics hub | `spacecraft` | ❌ (Phase 1) |
| Gravity/world | `simIncludeGravBody`, `planetEphemeris`, SPICE | ❌ (Phase 1→2) |
| Reaction wheels | `reactionWheelStateEffector` | ❌ (Phase 1) |
| Thrusters | `thrusterStateEffector` | ❌ (Phase 4) |
| MTB | `MtbEffector` | ❌ (Phase 4) |
| SRP | `facetSRPDynamicEffector`, `radiationPressure` | ❌ (Phase 2) |
| Drag | `dragDynamicEffector`, `facetDragDynamicEffector` | ❌ (Phase 2) |
| Magnetic field | `magneticFieldWMM`, `magneticFieldCenteredDipole` | ❌ (Phase 2) |
| Atmosphere | `MsisAtmosphere`, `ExponentialAtmosphere` | ❌ (Phase 2) |
| Eclipse | `eclipse` | ❌ (Phase 2) |
| Sensors (att) | `coarseSunSensor`, `starTracker`, `imuSensor`, `magnetometer` | ❌ (Phase 3) |
| Estimation | `simpleNav`, `InertialUKF`, `CSSEst` | ❌ (Phase 3) |
| Att guidance | `inertial3D`, `velocityPoint`, `hillPoint` | ❌ (Phase 1→5) |
| Att control | `mrpFeedback`, `mrpPD` | ❌ (Phase 1→5) |
| Att effector iface | `rwMotorTorque`, `thrForceMapping` | ❌ (Phase 1→4) |
| Formation | `formationBarycenter`, `hillStateConverter` | ❌ (Phase 6) |
| Comms | `simpleAntenna`, `linkBudget` | ❌ (Phase 6) |
| Monte Carlo | `MonteCarlo.Controller` | ❌ (Phase 6) |
| Visualization | `vizSupport` | ❌ (Phase 1) |
| Data export | `recorder`, `PythonVariableLogger` | ❌ (Phase 1) |
