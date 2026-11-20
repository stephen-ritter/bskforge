# `bskforge` scenario format reference

Generated from the pydantic schema (v2) — every field, type and default that `bskforge validate` accepts. Regenerate with `bskforge reference > REFERENCE.md`.

Top-level tables: `[simulation]`, `[world]`, `[environment]`, `[spacecraft]`, `[comm_links]`, `[formations]`, `[logging]`, `[visualization]`, `[custom]`, `[monte_carlo]` (`[[spacecraft]]`, `[[comm_links]]` and `[[formations]]` are repeatable).

| table | type | description |
| --- | --- | --- |
| `[simulation]` | `SimulationSection` |  |
| `[world]` | `WorldSection` |  |
| `[environment]` | `EnvironmentSection` / `null` |  |
| `[spacecraft]` | array[table] |  |
| `[comm_links]` | array[`type` ∈ {`crosslink`, `downlink`}] |  |
| `[formations]` | array[table] |  |
| `[logging]` | `LoggingSection` |  |
| `[visualization]` | `VisualizationSection` |  |
| `[custom]` | `CustomSection` |  |
| `[monte_carlo]` | `MonteCarloSection` / `null` |  |

## `[simulation]`
[simulation]: global run parameters.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `name` | str | `'bskforge_scenario'` |  |  |
| `duration` | float | `600.0` |  | Simulated duration [s] |
| `dt` | `DtConfig` |  |  |  |

### `[simulation.dt]`
Task rates for the simulation processes.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `dynamics` | float | `0.1` |  | Dynamics task rate [s] |
| `fsw` | float | `1.0` |  | FSW task rate [s] |

## `[world]`
[world]: gravitational environment shared by all vehicles.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `central_body` | str | `'earth'` |  |  |
| `gravities` | array[table] |  |  |  |
| `epoch` | str / `null` | `None` |  |  |
| `spice` | `SpiceConfig` |  |  |  |
| `sun` | `SunConfig` |  |  |  |

### `[[world.gravities]]`
A gravitating body: [[world.gravities]].

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `params` | table | `{}` |  |  |
| `name` | str |  | ✅ |  |
| `type` | ``planet` / `custom`` | `'planet'` |  |  |
| `central` | bool | `False` |  |  |
| `mu` | float / `null` | `None` |  | Gravitational parameter [m^3/s^2] (custom) |
| `radius` | float / `null` | `None` |  | Equatorial radius [m] (custom) |
| `model_dictionary_key` | str / `null` | `None` |  |  |
| `grav_order` | int / `null` | `None` |  | Spherical-harmonics order [-] |
| `grav_file` | str / `null` | `None` |  | LocalGravData support-file key |

### `[world.spice]`
SPICE ephemerides: [world.spice].

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `enabled` | bool | `False` |  |  |
| `kernels` | array[str] |  |  |  |
| `zero_base` | str / `null` | `None` |  | Origin body name |

### `[world.sun]`
Sun source for sensors and SRP.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `position` | array[float] | `[149600000000.0, 0.0, 0.0]` |  |  |

## `[environment]`
[environment]: environment models applied to every vehicle.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `magnetic_field` | `MagneticFieldConfig` / `null` | `None` |  |  |
| `atmosphere` | `AtmosphereConfig` / `null` | `None` |  |  |
| `eclipse` | `EclipseConfig` / `null` | `None` |  |  |
| `solar_flux` | float / `null` | `None` |  |  |

### `[environment.magnetic_field]`
Planetary magnetic field: [environment.magnetic_field].

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `params` | table | `{}` |  |  |
| `type` | ``centered_dipole` / `wmm`` | `'centered_dipole'` |  |  |

### `[environment.atmosphere]`
Planetary atmosphere: [environment.atmosphere].

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `params` | table | `{}` |  |  |
| `type` | ``exponential` / `msis`` | `'exponential'` |  |  |
| `ap` | float | `4.0` |  | Daily Ap index (MSIS) |
| `f107` | float | `150.0` |  | F10.7 flux (MSIS) |

### `[environment.eclipse]`
Planet shadow model: [environment.eclipse].

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `params` | table | `{}` |  |  |
| `type` | str | `'penumbra'` |  |  |

## `[[spacecraft]]`
[[spacecraft]]: one complete space vehicle.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `name` | str |  | ✅ |  |
| `hub` | `HubConfig` |  |  |  |
| `orbit` | `type` ∈ {`elements`, `rv`} | `None` |  |  |
| `attitude` | `AttitudeConfig` / `null` | `None` |  |  |
| `sensors` | array[`type` ∈ {`camera`, `coarseSunSensor`, `imuSensor`, `magnetometer`, `starTracker`}] |  |  |  |
| `css_constellations` | array[table] |  |  |  |
| `effectors` | array[`type` ∈ {`mtb`, `reactionWheels`, `thrusters`}] |  |  |  |
| `disturbances` | array[`type` ∈ {`drag`, `gravityGradient`, `solarRadiationPressure`}] |  |  |  |
| `fsw` | `FswSection` / `null` | `None` |  |  |
| `environment` | `EnvironmentSection` / `null` | `None` |  |  |
| `logging` | `SpacecraftLogging` |  |  |  |
| `viz` | `SpacecraftViz` |  |  |  |

### `[spacecraft.hub]`
Rigid-body hub properties: [spacecraft.hub].

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `params` | table | `{}` |  |  |
| `mass` | float | `1.0` |  | mHub [kg] |
| `inertia` | table |  |  | IHubPntBc_B [kg*m^2] |
| `r_BcB_B` | array[float] | `[0.0, 0.0, 0.0]` |  |  |

### `[spacecraft.attitude]`
[spacecraft.attitude]: rotational initial conditions.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `initial` | `AttitudeInitial` / `null` | `None` |  |  |

### `[spacecraft.attitude.initial]`
Initial attitude and rate state.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `sigma_BN` | array[float] | `[0.0, 0.0, 0.0]` |  |  |
| `omega_BN_B` | array[float] | `[0.0, 0.0, 0.0]` |  |  |

### `[[spacecraft.sensors]]` — `coarseSunSensor`
Coarse sun sensor: type = "coarseSunSensor".

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `params` | table | `{}` |  |  |
| `type` | str | `'coarseSunSensor'` |  |  |
| `name` | str | `'css'` |  |  |
| `boresight_B` | array[float] | `[1.0, 0.0, 0.0]` |  |  |
| `fov_deg` | float | `30.0` |  | Half-angle FOV [deg] |
| `noise_sigma` | float / `null` | `None` |  | Output noise sigma [-] |
| `bias` | float / `null` | `None` |  | Output bias [-] |
| `recorder` | `RecorderConfig` |  |  |  |

### `[[spacecraft.sensors]]` — `starTracker`
Star tracker: type = "starTracker".

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `params` | table | `{}` |  |  |
| `type` | str | `'starTracker'` |  |  |
| `name` | str | `'st'` |  |  |
| `sigma_SB` | array[float] | `[0.0, 0.0, 0.0]` |  |  |
| `noise_sigma` | float / `null` | `None` |  | Random-walk sigma [rad] |
| `white_noise` | float / `null` | `None` |  | White-noise sigma [rad] |
| `recorder` | `RecorderConfig` |  |  |  |

### `[[spacecraft.sensors]]` — `imuSensor`
Inertial measurement unit: type = "imuSensor".

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `params` | table | `{}` |  |  |
| `type` | str | `'imuSensor'` |  |  |
| `name` | str | `'imu'` |  |  |
| `sigma_PB` | array[float] | `[0.0, 0.0, 0.0]` |  |  |
| `gyro_bias` | array[float] | `[0.0, 0.0, 0.0]` |  |  |
| `accel_bias` | array[float] | `[0.0, 0.0, 0.0]` |  |  |
| `gyro_walk` | array[float] / `null` | `None` |  | Gyro random walk |
| `accel_walk` | array[float] / `null` | `None` |  | Accel random walk |
| `recorder` | `RecorderConfig` |  |  |  |

### `[[spacecraft.sensors]]` — `magnetometer`
Magnetometer: type = "magnetometer".

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `params` | table | `{}` |  |  |
| `type` | str | `'magnetometer'` |  |  |
| `name` | str | `'magnetometer'` |  |  |
| `sigma_SB` | array[float] | `[0.0, 0.0, 0.0]` |  |  |
| `noise_sigma` | float / `null` | `None` |  | Noise sigma [T] |
| `bias` | array[float] / `null` | `None` |  | Sensor bias [T] |
| `saturate` | bool | `False` |  |  |
| `recorder` | `RecorderConfig` |  |  |  |

### `[[spacecraft.sensors]]` — `camera`
Pinhole landmark camera: type = "camera".

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `params` | table | `{}` |  |  |
| `type` | str | `'camera'` |  |  |
| `name` | str | `'camera'` |  |  |
| `sigma_CB` | array[float] | `[0.0, 0.0, 0.0]` |  |  |
| `focal_length` | float | `0.025` |  | Focal length f [m] |
| `pixel_size` | float | `1.1e-05` |  | Pixel pitch [m] |
| `resolution` | array[int] |  |  |  |
| `target` | str / `null` | `None` |  | Target body name |
| `landmarks` | array[table] |  |  |  |
| `sun_mask_angle` | float / `null` | `None` |  | maskSun [rad] |
| `recorder` | `RecorderConfig` |  |  |  |

### `[[spacecraft.sensors.landmarks]]`
A fixed landmark on the camera's target body.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `position` | array[float] |  | ✅ |  |
| `normal` | array[float] | `[1.0, 0.0, 0.0]` |  |  |

### `[[spacecraft.sensors]].recorder` — used by `coarseSunSensor`, `starTracker`, `imuSensor`, `magnetometer`, `camera`
Recorder settings for a single message stream.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `enabled` | bool | `True` |  |  |
| `sample_time` | float / `null` | `None` |  | Recording sample period [s] (auto if unset) |

### `[[spacecraft.css_constellations]]`
Aggregates several CSS sensors into one sunline measurement.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `name` | str | `'css'` |  |  |
| `members` | array[str] |  | ✅ |  |

### `[[spacecraft.effectors]]` — `reactionWheels`
Reaction-wheel set: type = "reactionWheels".

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `params` | table | `{}` |  |  |
| `type` | str | `'reactionWheels'` |  |  |
| `wheels` | array[table] |  |  |  |
| `inertia` | float | `0.01` |  | Spin inertia Js [kg*m^2] |
| `recorder` | `RecorderConfig` |  |  |  |

### `[[spacecraft.effectors.wheels]]`
A single reaction wheel device.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `params` | table | `{}` |  |  |
| `name` | str / `null` | `None` |  |  |
| `direction_B` | array[float] | `[1.0, 0.0, 0.0]` |  |  |
| `max_speed` | float | `5500.0` |  | Max speed [rad/s] |
| `max_torque` | float / `null` | `None` |  | Motor torque limit [Nm] |
| `jitter_model` | ``none` / `constant` / `speedDependent`` | `'none'` |  |  |
| `initial_speed` | float | `0.0` |  | Initial wheel speed [rpm] |

### `[spacecraft.effectors.recorder]`
Recorder settings for a single message stream.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `enabled` | bool | `True` |  |  |
| `sample_time` | float / `null` | `None` |  | Recording sample period [s] (auto if unset) |

### `[[spacecraft.effectors]]` — `thrusters`
Thruster set: type = "thrusters".

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `params` | table | `{}` |  |  |
| `type` | str | `'thrusters'` |  |  |
| `thrusters` | array[table] |  |  |  |

### `[[spacecraft.effectors.thrusters]]`
A single thruster device.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `params` | table | `{}` |  |  |
| `name` | str / `null` | `None` |  |  |
| `direction_B` | array[float] | `[1.0, 0.0, 0.0]` |  |  |
| `position_B` | array[float] | `[0.0, 0.0, 0.0]` |  |  |
| `max_thrust` | float / `null` | `None` |  | Peak force [N] |
| `model` | str | `'Blank_Thruster'` |  |  |

### `[[spacecraft.effectors]]` — `mtb`
Magnetic torque-bar set: type = "mtb".

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `params` | table | `{}` |  |  |
| `type` | str | `'mtb'` |  |  |
| `bars` | array[table] |  |  |  |

### `[[spacecraft.effectors.bars]]`
A single magnetic torque bar.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `params` | table | `{}` |  |  |
| `name` | str / `null` | `None` |  |  |
| `direction_B` | array[float] | `[1.0, 0.0, 0.0]` |  |  |
| `max_dipole` | float | `0.12` |  | Peak dipole [A*m^2] |

### `[[spacecraft.disturbances]]` — `gravityGradient`
Gravity-gradient torque: type = "gravityGradient".

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `params` | table | `{}` |  |  |
| `type` | str | `'gravityGradient'` |  |  |

### `[[spacecraft.disturbances]]` — `drag`
Atmospheric drag: type = "drag".

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `params` | table | `{}` |  |  |
| `type` | str | `'drag'` |  |  |
| `model` | ``cannonball` / `faceted`` | `'cannonball'` |  |  |
| `area` | float | `20.0` |  | Projected area [m^2] |
| `drag_coeff` | float | `2.2` |  | Drag coeff [-] |
| `facets` | array[table] |  |  |  |

### `[[spacecraft.disturbances.facets]]`
One flat plate of a faceted drag geometry.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `area` | float |  | ✅ | Facet area [m^2] |
| `drag_coeff` | float | `2.2` |  | Facet drag coeff [-] |
| `normal_B` | array[float] |  | ✅ |  |
| `location_B` | array[float] | `[0.0, 0.0, 0.0]` |  |  |

### `[[spacecraft.disturbances]]` — `solarRadiationPressure`
Solar radiation pressure: type = "solarRadiationPressure".

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `params` | table | `{}` |  |  |
| `type` | str | `'solarRadiationPressure'` |  |  |
| `model` | ``cannonball` / `faceted`` | `'cannonball'` |  |  |
| `area` | float | `20.0` |  | Cannonball area [m^2] |
| `reflection_coeff` | float | `1.3` |  | coefficientReflection [-] |
| `facets` | array[table] |  |  |  |

### `[[spacecraft.disturbances.facets]]`
One facet of a faceted SRP geometry.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `area` | float |  | ✅ | Facet area [m^2] |
| `nHat_B` | array[float] |  | ✅ |  |
| `location_B` | array[float] | `[0.0, 0.0, 0.0]` |  |  |
| `diffuse_coeff` | float | `0.2` |  |  |
| `specular_coeff` | float | `0.5` |  |  |

### `[spacecraft.fsw]`
[spacecraft.fsw]: the per-vehicle flight-software stack.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `params` | table | `{}` |  |  |
| `guidance` | `type` ∈ {`celestialTwoBodyPoint`, `hillPoint`, `inertial3D`, `locationPointing`, `sunSafePoint`, `velocityPoint`, `waypointReference`} | `None` |  |  |
| `estimator` | `type` ∈ {`cssWlsEst`, `inertialUKF`, `simpleNav`, `sunlineSuKF`} | `None` |  |  |
| `control` | `type` ∈ {`mrpFeedback`, `mrpPD`, `rateServo`} | `None` |  |  |
| `allocation` | `type` ∈ {`dipoleMapping`, `rwMotorTorque`, `thrusterTorque`} | `None` |  |  |
| `momentum_management` | `type` ∈ {`mtbMomentumManagementSimple`, `thrMomentumDumping`} | `None` |  |  |
| `desaturation` | `type` ∈ {`rwNullSpace`} | `None` |  |  |

### `[spacecraft.environment]`
[environment]: environment models applied to every vehicle.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `magnetic_field` | `MagneticFieldConfig` / `null` | `None` |  |  |
| `atmosphere` | `AtmosphereConfig` / `null` | `None` |  |  |
| `eclipse` | `EclipseConfig` / `null` | `None` |  |  |
| `solar_flux` | float / `null` | `None` |  |  |

### `[spacecraft.environment.magnetic_field]`
Planetary magnetic field: [environment.magnetic_field].

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `params` | table | `{}` |  |  |
| `type` | ``centered_dipole` / `wmm`` | `'centered_dipole'` |  |  |

### `[spacecraft.environment.atmosphere]`
Planetary atmosphere: [environment.atmosphere].

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `params` | table | `{}` |  |  |
| `type` | ``exponential` / `msis`` | `'exponential'` |  |  |
| `ap` | float | `4.0` |  | Daily Ap index (MSIS) |
| `f107` | float | `150.0` |  | F10.7 flux (MSIS) |

### `[spacecraft.environment.eclipse]`
Planet shadow model: [environment.eclipse].

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `params` | table | `{}` |  |  |
| `type` | str | `'penumbra'` |  |  |

### `[spacecraft.logging]`
Per-message recorder switches: [spacecraft.logging].

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `state` | `RecorderConfig` |  |  |  |
| `control_torque` | `RecorderConfig` |  |  |  |
| `rw_speeds` | `RecorderConfig` / `null` | `None` |  |  |

### `[spacecraft.logging.state]`
Recorder settings for a single message stream.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `enabled` | bool | `True` |  |  |
| `sample_time` | float / `null` | `None` |  | Recording sample period [s] (auto if unset) |

### `[spacecraft.logging.control_torque]`
Recorder settings for a single message stream.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `enabled` | bool | `True` |  |  |
| `sample_time` | float / `null` | `None` |  | Recording sample period [s] (auto if unset) |

### `[spacecraft.logging.rw_speeds]`
Recorder settings for a single message stream.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `enabled` | bool | `True` |  |  |
| `sample_time` | float / `null` | `None` |  | Recording sample period [s] (auto if unset) |

### `[spacecraft.viz]`
Per-vehicle Vizard appearance hints: [spacecraft.viz].

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `model_key` | str / `null` | `None` |  |  |

## `[[comm_links]]` — `downlink`
Spacecraft-to-ground RF link: type = "downlink".

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `type` | str | `'downlink'` |  |  |
| `name` | str |  | ✅ |  |
| `transmitter` | str |  | ✅ | Transmitting spacecraft name |
| `ground_station` | `GroundStationConfig` |  | ✅ |  |
| `tx_antenna` | `AntennaConfig` |  |  |  |
| `rx_antenna` | `AntennaConfig` |  |  |  |

### `[comm_links.ground_station]`
A site on the central body (groundLocation.GroundLocation).

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `lat` | float |  | ✅ | Latitude [deg] |
| `long` | float |  | ✅ | East longitude [deg] |
| `elevation` | float | `0.0` |  | Elevation [m] |
| `min_elevation` | float | `0.0` |  | Min access elevation [deg] |

### `[comm_links.tx_antenna]`
One RF antenna end of a link (simpleAntenna.SimpleAntenna).

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `frequency` | float | `8400000000.0` |  | Center frequency [Hz] |
| `bandwidth` | float | `20000000.0` |  | Bandwidth [Hz] |
| `directivity_db` | float | `20.0` |  | Directivity [dB] |
| `hpbw_ratio` | float | `1.0` |  | HPBW ratio k |
| `tx_power` | float | `100.0` |  | Transmit power [W] |
| `radial_efficiency` | float | `0.95` |  |  |
| `noise_temperature` | float | `290.0` |  | System noise temp [K] |
| `position_B` | array[float] | `[0.0, 0.0, 1.0]` |  |  |
| `orientation_B` | array[float] | `[0.0, 0.0, 0.0]` |  |  |
| `ambient_temperature` | float / `null` | `None` |  |  |

### `[comm_links.rx_antenna]`
One RF antenna end of a link (simpleAntenna.SimpleAntenna).

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `frequency` | float | `8400000000.0` |  | Center frequency [Hz] |
| `bandwidth` | float | `20000000.0` |  | Bandwidth [Hz] |
| `directivity_db` | float | `20.0` |  | Directivity [dB] |
| `hpbw_ratio` | float | `1.0` |  | HPBW ratio k |
| `tx_power` | float | `100.0` |  | Transmit power [W] |
| `radial_efficiency` | float | `0.95` |  |  |
| `noise_temperature` | float | `290.0` |  | System noise temp [K] |
| `position_B` | array[float] | `[0.0, 0.0, 1.0]` |  |  |
| `orientation_B` | array[float] | `[0.0, 0.0, 0.0]` |  |  |
| `ambient_temperature` | float / `null` | `None` |  |  |

## `[[comm_links]]` — `crosslink`
Spacecraft-to-spacecraft RF link: type = "crosslink".

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `type` | str | `'crosslink'` |  |  |
| `name` | str |  | ✅ |  |
| `transmitter` | str |  | ✅ | Transmitting spacecraft name |
| `receiver` | str |  | ✅ | Receiving spacecraft name |
| `tx_antenna` | `AntennaConfig` |  |  |  |
| `rx_antenna` | `AntennaConfig` |  |  |  |

### `[comm_links.tx_antenna]`
One RF antenna end of a link (simpleAntenna.SimpleAntenna).

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `frequency` | float | `8400000000.0` |  | Center frequency [Hz] |
| `bandwidth` | float | `20000000.0` |  | Bandwidth [Hz] |
| `directivity_db` | float | `20.0` |  | Directivity [dB] |
| `hpbw_ratio` | float | `1.0` |  | HPBW ratio k |
| `tx_power` | float | `100.0` |  | Transmit power [W] |
| `radial_efficiency` | float | `0.95` |  |  |
| `noise_temperature` | float | `290.0` |  | System noise temp [K] |
| `position_B` | array[float] | `[0.0, 0.0, 1.0]` |  |  |
| `orientation_B` | array[float] | `[0.0, 0.0, 0.0]` |  |  |
| `ambient_temperature` | float / `null` | `None` |  |  |

### `[comm_links.rx_antenna]`
One RF antenna end of a link (simpleAntenna.SimpleAntenna).

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `frequency` | float | `8400000000.0` |  | Center frequency [Hz] |
| `bandwidth` | float | `20000000.0` |  | Bandwidth [Hz] |
| `directivity_db` | float | `20.0` |  | Directivity [dB] |
| `hpbw_ratio` | float | `1.0` |  | HPBW ratio k |
| `tx_power` | float | `100.0` |  | Transmit power [W] |
| `radial_efficiency` | float | `0.95` |  |  |
| `noise_temperature` | float | `290.0` |  | System noise temp [K] |
| `position_B` | array[float] | `[0.0, 0.0, 1.0]` |  |  |
| `orientation_B` | array[float] | `[0.0, 0.0, 0.0]` |  |  |
| `ambient_temperature` | float / `null` | `None` |  |  |

## `[[formations]]`
[[formations]]: a named vehicle group with relative states.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `name` | str |  | ✅ |  |
| `members` | array[str] |  | ✅ |  |
| `relative_to` | str / `null` | `None` |  |  |
| `barycenter` | bool | `True` |  |  |

## `[logging]`
[logging]: recording defaults and data export.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `directory` | str / `null` | `None` |  |  |
| `format` | ``csv` / `parquet` / `npz`` | `'csv'` |  |  |
| `default_num_points` | int | `1000` |  |  |

## `[visualization]`
[visualization]: Vizard live/replay visualization settings.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `enabled` | bool | `False` |  |  |
| `live_stream` | bool | `False` |  |  |
| `broadcast_stream` | bool | `False` |  |  |
| `save_file` | str / `null` | `None` |  |  |
| `no_display` | bool | `False` |  |  |

## `[custom]`
[custom]: direct Basilisk graph primitives.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `tasks` | array[table] |  |  |  |
| `modules` | array[table] |  |  |  |
| `connections` | array[table] |  |  |  |
| `events` | array[table] |  |  |  |
| `simulation_options` | table |  |  |  |

### `[[custom.tasks]]`
A Basilisk task on a (possibly new) process: [[custom.tasks]].

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `name` | str |  | ✅ |  |
| `rate` | float |  | ✅ | Task rate [s] |
| `process` | str | `'customProcess'` |  |  |
| `process_priority` | int | `1` |  |  |

### `[[custom.modules]]`
Any Basilisk module class: [[custom.modules]].

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `module` | str |  | ✅ |  |
| `tag` | str |  | ✅ |  |
| `task` | str / `null` | `None` |  |  |
| `priority` | int | `-1` |  |  |
| `params` | table |  |  |  |

### `[[custom.connections]]`
A message subscription: [[custom.connections]].

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `source` | str |  | ✅ |  |
| `destination` | str |  | ✅ |  |

### `[[custom.events]]`
A Basilisk event: [[custom.events]].

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `name` | str |  | ✅ |  |
| `rate` | float | `1.0` |  | Evaluation period [s] |
| `condition_time` | float / `null` | `None` |  | Trigger time [s] |
| `condition` | str / `null` | `None` |  |  |
| `effect` | str / `null` | `None` |  |  |
| `transition` | str | `'atExit'` |  |  |

## `[monte_carlo]`
[monte_carlo]: repeat the scenario with dispersed initial states.

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `runs` | int |  | ✅ |  |
| `threads` | int | `1` |  |  |
| `archive_dir` | str / `null` | `None` |  |  |
| `dispersions` | array[`type` ∈ {`mass`, `omega_BN_B`, `orbit_elements`, `sigma_BN`}] |  |  |  |

### `[[monte_carlo.dispersions]]` — `?`
Randomize the initial orbit: quantity = "orbit_elements".

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `quantity` | str | `'orbit_elements'` |  |  |
| `spacecraft` | str |  | ✅ |  |
| `elements` | table |  |  |  |

### `[[monte_carlo.dispersions]]` — `?`
Uniform random Euler-angle MRP: quantity = "sigma_BN".

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `quantity` | str | `'sigma_BN'` |  |  |
| `spacecraft` | str |  | ✅ |  |

### `[[monte_carlo.dispersions]]` — `?`
Gaussian initial body rate: quantity = "omega_BN_B".

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `quantity` | str | `'omega_BN_B'` |  |  |
| `spacecraft` | str |  | ✅ |  |
| `std` | array[float] |  | ✅ |  |
| `mean` | array[float] | `[0.0, 0.0, 0.0]` |  |  |

### `[[monte_carlo.dispersions]]` — `?`
Uniform hub mass: quantity = "mass".

| field | type | default | required | description |
| --- | --- | --- | --- | --- |
| `quantity` | str | `'mass'` |  |  |
| `spacecraft` | str |  | ✅ |  |
| `range` | array[float] |  | ✅ |  |
