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
semi_major_axis = 1.0e7
eccentricity = 0.01
inclination = 33.3

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
