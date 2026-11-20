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
bskforge mc examples/monte_carlo.toml    # [monte_carlo] dispersion campaign
bskforge validate scenario.toml
bskforge schema                    # dump the JSON schema of the scenario format
bskforge reference                 # field-by-field format reference (generated)
bskforge graph scenario.toml       # emit the Basilisk message-connection graph
```

Every field, type and default of the scenario format is documented in
[`REFERENCE.md`](REFERENCE.md), generated straight from the validating
pydantic schema (`bskforge reference > REFERENCE.md`) — the docs cannot
drift from the code. `bskforge reference --toml` prints a commented TOML
skeleton of the whole format to start a new scenario from.

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

## Examples

Every example is executed by the test suite:

| file | shows |
| --- | --- |
| `single_sat_leo.toml` | minimal orbit + reaction-wheel attitude control |
| `single_sat_with_sensors.toml` | CSS constellation, star tracker, IMU logging |
| `earth_environment.toml` | SPICE world, WMM/MSIS/eclipse, drag/SRP/gravity-gradient |
| `estimation_and_opnav.toml` | CSS-cluster WLS + inertial UKF estimation, landmark camera |
| `momentum_management.toml` | thruster and MTB momentum dumping, RW null-space |
| `guidance_laws.toml` | `velocityPoint`/`hillPoint` guidance with rateServo/mrpPD |
| `ground_pointing.toml` | `locationPointing` over a ground site (SPICE planet) |
| `comm_links.toml` | ground-station downlink + crosslink RF link budgets |
| `relay_chain.toml` | crosslink → downlink relay chain with formation states |
| `formation_flying.toml` | barycenter + Hill-frame relative states |
| `multi_sat_formation.toml` | two-vehicle chase scenario |
| `monte_carlo.toml` | dispersed initial states (`bskforge mc`) |
| `custom_escape_hatch.toml` | native Basilisk module wired in from TOML |

## Status (Phases 1–7: core + environment + sensors/estimation + effectors + FSW + fleets + docs)

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
multi-spacecraft scenarios, RF comm links (spacecraft/ground antennas
with a full `linkBudget` chain) and formation flying (mass-weighted
barycenter plus Hill-frame relative states), message recorders with
CSV/parquet/npz export, Vizard visualization hooks, Monte Carlo
dispersion campaigns (`bskforge mc`), and the `[custom]` escape hatch.

Every documented block fails loudly (never silently) when it references
spacecraft, links or modules that do not exist.

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

## Upgrading Basilisk

bskforge is built so that most upstream changes need no bskforge code:

1. **New parameters on existing modules are free.** Every domain object
   accepts a `params = { ... }` table validated at build time against the
   *installed* Basilisk module by introspection, so new upstream knobs
   work from TOML immediately (with did-you-mean errors for typos).
2. **New modules are free via `[custom]`.** Any class reachable by import
   path can be added as a `[custom.modules]` entry with
   `[custom.connections]` wiring — a new Basilisk feature is usable from
   day one. It becomes a first-class TOML section (schema class +
   builder + tests + example) only when deliberately promoted.
3. **Removed/renamed modules and changed messages are the only tier that
   forces work** — the flow below catches them.

Workflow for a Basilisk version bump:

1. Install the new wheel in the project venv and update the version pin
   in `pyproject.toml`.
2. Run the full sweep: `.venv/bin/pytest`. The tests build and fly real
   Basilisk graphs — including the example matrix — so removals fail at
   build (`ImportError`/`BuildError`), message-payload changes fail as
   missing recorder columns (`KeyError` in export/tests), and behavioral
   drift fails the physics asserts (attitude convergence, Hill
   separations, momentum budgets) — not just "did it run".
3. Triage each failure to its builder in `src/bskforge/build/*.py`
   (`BSK_ERROR` and `BuildError` messages name the module). Change the
   pydantic schema only if the *TOML format* itself must move: bump the
   schema version and note the migration in this README.
4. Regenerate the docs — `bskforge reference > REFERENCE.md` — since the
   field reference and `bskforge schema` are derived from the schema,
   they re-derive automatically after the fix.
5. Re-run the definition-of-done gates and record the result in
   `ROADMAP.md`: completed entries are annotated *validated against
   Basilisk X.Y*, so an upgrade adds a new dated validation line rather
   than rewriting history.

## License

ISC — Autonomous Vehicle Systems Lab, University of Colorado Boulder.
