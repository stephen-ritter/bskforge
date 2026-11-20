# bskforge

Forge and run [Basilisk](https://github.com/AVSLab/basilisk) spacecraft
simulations from declarative scenario files.

A `bskforge` scenario file (TOML) captures a complete Basilisk simulation —
world, environment, one or more space vehicles, their sensors, effectors and
flight-software stack, logging and visualization — and `bskforge` builds and
launches the corresponding Basilisk simulation graph.

```bash
pip install "bskforge[dev]"        # or: pip install -e ".[dev]"
bskforge run examples/single_sat_leo.toml
bskforge validate scenario.toml
bskforge schema                    # dump the JSON schema of the scenario format
bskforge graph scenario.toml       # emit the Basilisk message-connection graph
```

## Scenario format (v2)

```toml
[simulation]
name = "LEO detumble"
duration = 600.0
dt = { dynamics = 0.1, fsw = 1.0 }

[world]
central_body = "earth"

[[spacecraft]]
name = "sc1"

[spacecraft.hub]
mass = 750.0
inertia = [900, 0, 0, 0, 800, 0, 0, 0, 600]   # kg*m^2, row-major

[spacecraft.orbit]
type = "elements"
[spacecraft.orbit.elements]
a = 1.0e7
e = 0.01
i = 33.3

[[spacecraft.effectors]]
type = "reactionWheels"
inertia = 0.01
[[spacecraft.effectors.wheels]]
direction_B = [1, 0, 0]

[[spacecraft.fsw.control]]
type = "mrpFeedback"
k = 3.5
p = 30.0

[spacecraft.logging]
state = { enabled = true, sample_time = 0.5 }
```

Multiple vehicles are declared with additional `[[spacecraft]]` tables; global
`[world]` / `[environment]` defaults can be overridden per vehicle. Scenario
files can inherit from another file with `extends = "base.toml"`.

## Guaranteed coverage: the `[custom]` escape hatch

Anything the domain sections do not model yet is still expressible: the
`[custom]` section exposes Basilisk's native graph primitives directly —
arbitrary module classes by import path, tasks/processes, message
connections (`"tag.outMsgName"` → `"tag.inMsgName"`), and events (time
conditions or `pkg.module:function` hooks). Every Basilisk scenario script
is a composition of those primitives, so the format can express everything
the installed Basilisk build can do. See
[`examples/custom_escape_hatch.toml`](examples/custom_escape_hatch.toml).

Every Basilisk object created by the domain builders also accepts a free-form
`params = { ... }` table; names are checked against the *installed* Basilisk
module at build time (with did-you-mean errors), so new upstream parameters
need no bskforge changes.

## Status (Phases 1–5: core + environment + sensors/estimation + effectors + FSW breadth)

Implemented: TOML loading with `extends` inheritance and friendly errors,
SPICE-driven worlds (presets, custom point masses, spherical harmonics,
third-body perturbations, sun-centered frames), hub + orbit/attitude initial
state, reaction-wheel effectors, CSS / star tracker / IMU / magnetometer /
pinhole landmark-camera sensors, CSS constellations, magnetic field (dipole,
WMM), atmosphere (exponential, MSIS), penumbra eclipse, gravity-gradient /
drag / SRP disturbances (cannonball and faceted), `inertial3D` →
`simpleNav`/`cssWlsEst`/`inertialUKF`/`sunlineSuKF` → `attTrackingError` →
`mrpFeedback`/`mrpPD`/`rateServo` → `rwMotorTorque` FSW stacks, seven
guidance laws (`inertial3D`, `velocityPoint`, `hillPoint`, `sunSafePoint`,
`locationPointing`, `celestialTwoBodyPoint`, `waypointReference`), thruster
sets with force-mapping and Schmitt/round firing, magnetic torque bars,
reaction-wheel momentum management (`thrMomentumDumping`,
`mtbMomentumManagementSimple`) and null-space desaturation,
multi-spacecraft scenarios, message recorders with CSV/parquet/npz export,
Vizard visualization hooks, and the `[custom]` escape hatch.

Pending: comms, formation FSW and Monte Carlo (Phase 6). Configuring any
pending block fails with a clear phase pointer instead of silently
ignoring it.

Known Basilisk behavior: recorder samples taken at `t = 0` capture messages
before their writer modules first execute (Basilisk logs them as
"not properly initialized" warnings once per stream); all later samples are
full-rate data.

See [`ROADMAP.md`](ROADMAP.md) for the Basilisk coverage matrix and what is
implemented at each phase.

## Development

```bash
python -m venv .venv && .venv/bin/pip install -e ".[dev]"
.venv/bin/pytest              # unit + integration tests (skips if Basilisk missing)
.venv/bin/ruff check src tests
.venv/bin/ruff format --check src tests
.venv/bin/ty check
```

Definition of done for anything tracked in `ROADMAP.md`: tests pass,
`ruff check`/`ruff format` clean, `ty check` clean.

## License

ISC — Autonomous Vehicle Systems Lab, University of Colorado Boulder.
