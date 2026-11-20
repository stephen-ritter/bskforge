#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for
#  any purpose with or without fee is hereby granted, provided that the
#  above copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

"""Unit tests for bsk_toml_sim models and parser."""

from pathlib import Path

import pytest

from bsk_toml_sim.models.vector import Vector3D, InertiaMatrix
from bsk_toml_sim.models.orbit import (
    ClassicalOrbitElements,
    OrbitConfig,
    RVState,
)
from bsk_toml_sim.models.attitude import MRPFeedbackConfig
from bsk_toml_sim.models.sensors import (
    CameraConfig,
    SunSensorConfig,
    MagnetometerConfig,
    StarTrackerConfig,
    IMUConfig,
    SensorConfig,
)
from bsk_toml_sim.models.effectors import (
    SingleReactionWheelConfig,
    ReactionWheelSetConfig,
    ThrusterConfig,
    SRPEffectorConfig,
    DragEffectorConfig,
    EffectorConfig,
)
from bsk_toml_sim.models.spacecraft import HubConfig, SpacecraftConfig
from bsk_toml_sim.models.gravity import GravityBodyConfig, CentralBodyConfig
from bsk_toml_sim.models.recorder import RecorderConfig
from bsk_toml_sim.models.simulation import SimulationConfig
from bsk_toml_sim.parser import load_simulation_config


# ---------------------------------------------------------------------------
# Vector3D tests
# ---------------------------------------------------------------------------

class TestVector3D:
    """Tests for the Vector3D model."""

    def test_default_values(self):
        """Vector3D defaults to zero."""
        v = Vector3D()
        assert (v.x, v.y, v.z) == (0.0, 0.0, 0.0)

    def test_from_dict(self):
        """Vector3D from explicit dict."""
        v = Vector3D(x=1.0, y=2.0, z=3.0)
        assert v.to_list() == [1.0, 2.0, 3.0]

    def test_to_column_matrix(self):
        """Vector3D to_column_matrix returns Eigen-style list."""
        v = Vector3D(x=1.0, y=2.0, z=3.0)
        result = v.to_column_matrix()
        assert result == [[1.0], [2.0], [3.0]]


# ---------------------------------------------------------------------------
# InertiaMatrix tests
# ---------------------------------------------------------------------------

class TestInertiaMatrix:
    """Tests for the InertiaMatrix model."""

    def test_diagonal_defaults_off_diagonal_to_zero(self):
        """Default off-diagonals are zero."""
        i = InertiaMatrix(ixx=100.0, iyy=200.0, izz=300.0)
        assert i.ixy == 0.0
        assert i.ixz == 0.0
        assert i.iyz == 0.0

    def test_to_flat_produces_9_elements(self):
        """to_flat returns a 9-element tuple."""
        i = InertiaMatrix(ixx=1.0, iyy=2.0, izz=3.0)
        flat = i.to_flat()
        assert len(flat) == 9
        assert flat == (1.0, 0.0, 0.0, 0.0, 2.0, 0.0,
                        0.0, 0.0, 3.0)

    def test_to_flat_includes_off_diagonal(self):
        """Off-diagonal terms appear symmetrically."""
        i = InertiaMatrix(ixx=1.0, iyy=2.0, izz=3.0,
                          ixy=0.5, ixz=0.3, iyz=0.7)
        flat = i.to_flat()
        # Column-major: [Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz]
        assert flat == (1.0, 0.5, 0.3, 0.5, 2.0, 0.7,
                        0.3, 0.7, 3.0)


# ---------------------------------------------------------------------------
# Orbit tests
# ---------------------------------------------------------------------------

class TestOrbitConfig:
    """Tests for orbit configuration models."""

    def test_classical_elements_default(self):
        """ClassicalOrbitElements has sensible defaults."""
        oe = ClassicalOrbitElements()
        assert oe.semi_major_axis == 7000e3
        assert oe.eccentricity == 0.0

    def test_orbit_config_elements_mode(self):
        """OrbitConfig with type='elements' works."""
        cfg = OrbitConfig(
            type="elements",
            elements=ClassicalOrbitElements(
                semi_major_axis=7000e3,
                inclination=45.0,
            ),
        )
        elems = cfg.get_elements()
        assert elems.semi_major_axis == 7000e3
        assert elems.inclination == 45.0

    def test_orbit_config_rv_mode(self):
        """OrbitConfig with type='rv' works."""
        cfg = OrbitConfig(
            type="rv",
            rv=RVState(
                position=Vector3D(x=6730e3, y=0.0, z=0.0),
                velocity=Vector3D(x=0.0, y=3.87e3, z=0.0),
            ),
        )
        rv = cfg.get_rv()
        assert rv.position.x == 6730e3

    def test_wrong_mode_raises(self):
        """Accessing wrong mode raises ValueError."""
        cfg = OrbitConfig(type="elements")
        with pytest.raises(ValueError):
            cfg.get_rv()


# ---------------------------------------------------------------------------
# Attitude tests
# ---------------------------------------------------------------------------

class TestAttitudeConfig:
    """Tests for attitude configuration models."""

    def test_default_mrp_feedback(self):
        """MRP feedback defaults are reasonable."""
        cfg = MRPFeedbackConfig()
        assert cfg.k == 3.5
        assert cfg.p == 30.0
        assert cfg.ki == -1.0

    def test_integral_limit_computed_when_ki_positive(self):
        """Integral limit is computed from Ki when positive."""
        cfg = MRPFeedbackConfig(ki=0.0002)
        limit = cfg.compute_integral_limit()
        assert abs(limit - 2.0 / 0.0002 * 0.1) < 1e-9

    def test_integral_limit_zero_when_ki_negative(self):
        """Integral limit is zero when Ki is negative (disabled)."""
        cfg = MRPFeedbackConfig(ki=-1.0)
        assert cfg.compute_integral_limit() == 0.0

    def test_explicit_integral_limit_overrides(self):
        """Explicit integral_limit is returned as-is."""
        cfg = MRPFeedbackConfig(ki=0.001, integral_limit=5.0)
        assert cfg.compute_integral_limit() == 5.0


# ---------------------------------------------------------------------------
# Spacecraft tests
# ---------------------------------------------------------------------------

class TestSpacecraftConfig:
    """Tests for spacecraft configuration."""

    def test_minimal_spacecraft(self):
        """Minimal spacecraft with all defaults."""
        sc = SpacecraftConfig(name="test_sat")
        assert sc.name == "test_sat"
        assert sc.hub.mass == 1.0

    def test_spacecraft_with_hub(self):
        """Spacecraft with custom hub."""
        sc = SpacecraftConfig(
            name="heavy_sat",
            hub=HubConfig(
                mass=1500.0,
                inertia=InertiaMatrix(ixx=2000.0, iyy=1800.0, izz=1500.0),
            ),
        )
        assert sc.hub.mass == 1500.0
        assert sc.hub.inertia.ixx == 2000.0


# ---------------------------------------------------------------------------
# Gravity tests
# ---------------------------------------------------------------------------

class TestGravityConfig:
    """Tests for gravity body configuration."""

    def test_central_body_default(self):
        """CentralBodyConfig defaults to earth."""
        cfg = CentralBodyConfig()
        assert cfg.name == "earth"

    def test_gravity_body_with_central_flag(self):
        """GravityBodyConfig with is_central_body=True."""
        cfg = GravityBodyConfig(name="earth", is_central_body=True)
        assert cfg.is_central_body is True


# ---------------------------------------------------------------------------
# SimulationConfig tests
# ---------------------------------------------------------------------------

class TestSimulationConfig:
    """Tests for the top-level simulation config."""

    def test_defaults(self):
        """Default simulation config has sensible values."""
        cfg = SimulationConfig()
        assert cfg.name == "untitled_simulation"
        assert cfg.dt_dynamics == 0.1
        assert cfg.duration == 600.0

    def test_central_body_name_from_simple_config(self):
        """get_central_body_name uses CentralBodyConfig."""
        cfg = SimulationConfig(
            central_body=CentralBodyConfig(name="mars"),
        )
        assert cfg.get_central_body_name() == "mars"

    def test_central_body_name_from_gravity_list(self):
        """get_central_body_name falls back to gravity_bodies."""
        cfg = SimulationConfig(
            gravity_bodies=[
                GravityBodyConfig(name="sun"),
                GravityBodyConfig(name="earth", is_central_body=True),
            ],
        )
        assert cfg.get_central_body_name() == "earth"

    def test_central_body_name_fallback(self):
        """get_central_body_name returns 'earth' as final fallback."""
        cfg = SimulationConfig()
        assert cfg.get_central_body_name() == "earth"


# ---------------------------------------------------------------------------
# Parser / TOML integration tests
# ---------------------------------------------------------------------------

class TestParser:
    """Tests for TOML parsing via load_simulation_config."""

    @pytest.fixture
    def examples_dir(self) -> Path:
        """Path to the examples directory."""
        root = Path(__file__).resolve().parent.parent
        return root / "examples"

    def test_load_single_sat_example(self, examples_dir):
        """Load single_sat_leo.toml and verify structure."""
        cfg = load_simulation_config(examples_dir / "single_sat_leo.toml")
        assert cfg.name == "LEO Detumble"
        assert len(cfg.satellites) == 1
        sat = cfg.satellites[0]
        assert sat.name == "sc1"
        assert sat.hub.mass == 750.0
        assert sat.orbit.type == "elements"

    def test_load_multi_sat_example(self, examples_dir):
        """Load multi_sat_formation.toml and verify structure."""
        cfg = load_simulation_config(examples_dir / "multi_sat_formation.toml")
        assert cfg.name == "Two-Sat Formation"
        assert len(cfg.satellites) == 2
        names = [s.name for s in cfg.satellites]
        assert "chief" in names
        assert "deputy" in names

    def test_load_nonexistent_file_raises(self):
        """Loading a nonexistent file raises FileNotFoundError."""
        with pytest.raises(FileNotFoundError):
            load_simulation_config("/no/such/file.toml")

    def test_single_sat_orbit_elements_loaded(self, examples_dir):
        """Orbit elements are correctly parsed from TOML."""
        cfg = load_simulation_config(examples_dir / "single_sat_leo.toml")
        elems = cfg.satellites[0].orbit.get_elements()
        assert elems.semi_major_axis == 10000000.0
        assert abs(elems.eccentricity - 0.01) < 1e-9
        assert abs(elems.inclination - 33.3) < 1e-9

    def test_single_sat_attitude_control_loaded(self, examples_dir):
        """Attitude control gains are correctly parsed from TOML."""
        cfg = load_simulation_config(examples_dir / "single_sat_leo.toml")
        sat = cfg.satellites[0]
        assert sat.attitude is not None
        ctrl = sat.attitude.control
        assert ctrl is not None
        assert ctrl.mrp_feedback is not None
        assert ctrl.mrp_feedback.k == 3.5
        assert ctrl.mrp_feedback.p == 30.0


# ---------------------------------------------------------------------------
# Sensor tests
# ---------------------------------------------------------------------------

class TestCameraConfig:
    """Tests for camera sensor configuration."""

    def test_default_camera(self):
        """Camera defaults are sensible."""
        cfg = CameraConfig()
        assert cfg.name == "camera"
        assert cfg.field_of_view == 30.0
        assert cfg.image_width == 640
        assert cfg.is_on is True
        assert cfg.gaussian == 0.0

    def test_camera_with_corruption(self):
        """Camera with corruption parameters."""
        cfg = CameraConfig(
            name="imagingCam",
            sigma_CB=[0, 0, 1],
            gaussian=2.0,
            blur_param=3,
            dark_current=1.0,
        )
        assert cfg.gaussian == 2.0
        assert cfg.blur_param == 3


class TestSunSensorConfig:
    """Tests for sun sensor configuration."""

    def test_default_sun_sensor(self):
        """Sun sensor defaults."""
        cfg = SunSensorConfig()
        assert cfg.name == "sunSensor"
        assert cfg.sampling_time == 1.0

    def test_custom_sun_sensor(self):
        """Sun sensor with custom orientation."""
        cfg = SunSensorConfig(
            name="sunSensorX",
            sigma_SB=[1, 0, 0],
            sampling_time=0.5,
        )
        assert cfg.sigma_SB == [1, 0, 0]


class TestMagnetometerConfig:
    """Tests for magnetometer configuration."""

    def test_default_magnetometer(self):
        """Magnetometer defaults."""
        cfg = MagnetometerConfig()
        assert cfg.noise_sigma == 0.1
        assert cfg.bias == [0.0, 0.0, 0.0]


class TestStarTrackerConfig:
    """Tests for star tracker configuration."""

    def test_default_star_tracker(self):
        """Star tracker defaults."""
        cfg = StarTrackerConfig()
        assert cfg.noise_sigma == 0.001
        assert cfg.sampling_time == 2.0


class TestIMUConfig:
    """Tests for IMU configuration."""

    def test_default_imu(self):
        """IMU defaults."""
        cfg = IMUConfig()
        assert cfg.gyro_noise_sigma == 1e-4
        assert cfg.sampling_time == 0.01


# ---------------------------------------------------------------------------
# Effector tests
# ---------------------------------------------------------------------------

class TestReactionWheelConfig:
    """Tests for reaction wheel configuration."""

    def test_single_rw_default(self):
        """Single RW defaults."""
        cfg = SingleReactionWheelConfig()
        assert cfg.max_speed == 5500.0
        assert cfg.max_torque == -1.0
        assert cfg.jitter_model == "none"

    def test_rw_set_with_wheels(self):
        """RW set with multiple wheels."""
        cfg = ReactionWheelSetConfig(
            wheels=[
                SingleReactionWheelConfig(
                    name="RW-x", direction_B=[1, 0, 0]
                ),
                SingleReactionWheelConfig(
                    name="RW-y", direction_B=[0, 1, 0]
                ),
            ],
            inertia_rw=0.01,
        )
        assert len(cfg.wheels) == 2
        assert cfg.inertia_rw == 0.01


class TestThrusterConfig:
    """Tests for thruster configuration."""

    def test_default_thruster(self):
        """Thruster defaults."""
        cfg = ThrusterConfig()
        assert cfg.thrust_magnitude == 0.1
        assert cfg.isp == 200

    def test_custom_thruster(self):
        """Thruster with custom parameters."""
        cfg = ThrusterConfig(
            name="mainEngine",
            direction_B=[0, 0, -1],
            r_CMact_B=[0, 0, -0.5],
            thrust_magnitude=1.0,
            isp=300,
        )
        assert cfg.thrust_magnitude == 1.0
        assert cfg.r_CMact_B == [0, 0, -0.5]


class TestSRPEffectorConfig:
    """Tests for SRP effector configuration."""

    def test_default_srp(self):
        """SRP defaults."""
        cfg = SRPEffectorConfig()
        assert cfg.reflectivity_coeff == 0.3
        assert cfg.absorption_coeff == 0.7

    def test_srp_with_facets(self):
        """SRP with facet definition."""
        cfg = SRPEffectorConfig(
            facet_areas=[2.0, 2.0, 1.0],
            facet_normals_B=[
                [1, 0, 0], [-1, 0, 0], [0, 0, 1],
            ],
        )
        assert len(cfg.facet_areas) == 3


class TestDragEffectorConfig:
    """Tests for drag effector configuration."""

    def test_default_drag(self):
        """Drag defaults."""
        cfg = DragEffectorConfig()
        assert cfg.drag_coefficient == 2.2
        assert cfg.reference_area == 10.0


# ---------------------------------------------------------------------------
# Sensor/Effector container tests
# ---------------------------------------------------------------------------

class TestSensorConfig:
    """Tests for sensor container."""

    def test_empty_sensors(self):
        """Empty sensor config has no sensors."""
        cfg = SensorConfig()
        assert len(cfg.cameras) == 0
        assert len(cfg.sun_sensors) == 0

    def test_sensors_with_cameras(self):
        """Sensor config with cameras."""
        cfg = SensorConfig(
            cameras=[
                CameraConfig(name="cam1"),
                CameraConfig(name="cam2"),
            ],
            sun_sensors=[SunSensorConfig(name="sun1")],
        )
        assert len(cfg.cameras) == 2
        assert len(cfg.sun_sensors) == 1


class TestEffectorConfig:
    """Tests for effector container."""

    def test_empty_effectors(self):
        """Empty effector config defaults."""
        cfg = EffectorConfig()
        assert cfg.reaction_wheels is None
        assert len(cfg.thrusters) == 0

    def test_effectors_with_rws(self):
        """Effector config with reaction wheels."""
        cfg = EffectorConfig(
            reaction_wheels=ReactionWheelSetConfig(
                wheels=[SingleReactionWheelConfig(name="RW-x")]
            ),
            thrusters=[ThrusterConfig(name="thr1")],
        )
        assert cfg.reaction_wheels is not None
        assert len(cfg.reaction_wheels.wheels) == 1
        assert len(cfg.thrusters) == 1


# ---------------------------------------------------------------------------
# Spacecraft with sensors/effectors tests
# ---------------------------------------------------------------------------

class TestSpacecraftWithSensors:
    """Tests for spacecraft with sensors and effectors."""

    def test_spacecraft_defaults_have_empty_sensors(self):
        """Default spacecraft has empty sensor/effector configs."""
        sc = SpacecraftConfig(name="test")
        assert sc.sensors is not None
        assert sc.effectors is not None
        assert len(sc.sensors.cameras) == 0

    def test_spacecraft_with_full_config(self):
        """Spacecraft with sensors and effectors."""
        sc = SpacecraftConfig(
            name="full_sat",
            sensors=SensorConfig(
                cameras=[CameraConfig(name="cam1")],
                imus=[IMUConfig(name="imu")],
            ),
            effectors=EffectorConfig(
                reaction_wheels=ReactionWheelSetConfig(
                    wheels=[
                        SingleReactionWheelConfig(
                            name="RW-x", direction_B=[1, 0, 0]
                        )
                    ]
                ),
            ),
        )
        assert len(sc.sensors.cameras) == 1
        assert sc.sensors.cameras[0].name == "cam1"
        assert len(sc.sensors.imus) == 1
        assert sc.effectors.reaction_wheels is not None


# ---------------------------------------------------------------------------
# Parser tests for sensor/effector TOML
# ---------------------------------------------------------------------------

class TestParserWithSensors:
    """Tests for TOML parsing with sensors and effectors."""

    @pytest.fixture
    def examples_dir(self) -> Path:
        """Path to the examples directory."""
        root = Path(__file__).resolve().parent.parent
        return root / "examples"

    def test_load_sat_with_sensors(self, examples_dir):
        """Load TOML with sensors and verify parsing."""
        cfg = load_simulation_config(
            examples_dir / "single_sat_with_sensors.toml"
        )
        sat = cfg.satellites[0]

        # Cameras
        assert len(sat.sensors.cameras) == 2
        assert sat.sensors.cameras[0].name == "imagingCam"
        assert sat.sensors.cameras[0].gaussian == 2.0
        assert sat.sensors.cameras[1].name == "navCam"

        # Sun sensors
        assert len(sat.sensors.sun_sensors) == 2

        # Magnetometer
        assert len(sat.sensors.magnetometers) == 1
        assert sat.sensors.magnetometers[0].noise_sigma == 0.1

        # Star tracker
        assert len(sat.sensors.star_trackers) == 1

        # IMU
        assert len(sat.sensors.imus) == 1
        assert sat.sensors.imus[0].sampling_time == 0.01

    def test_load_sat_with_effectors(self, examples_dir):
        """Load TOML with effectors and verify parsing."""
        cfg = load_simulation_config(
            examples_dir / "single_sat_with_sensors.toml"
        )
        sat = cfg.satellites[0]

        # Reaction wheels
        assert sat.effectors.reaction_wheels is not None
        assert len(sat.effectors.reaction_wheels.wheels) == 3
        assert sat.effectors.reaction_wheels.wheels[0].name == "RW-x"
        assert sat.effectors.reaction_wheels.inertia_rw == 0.01

        # Thrusters
        assert len(sat.effectors.thrusters) == 1
        assert sat.effectors.thrusters[0].name == "thruster_z_neg"
        assert sat.effectors.thrusters[0].thrust_magnitude == 0.1

        # SRP
        assert sat.effectors.srp is not None
        assert len(sat.effectors.srp.facet_areas) == 6

        # Drag
        assert sat.effectors.drag is not None
        assert sat.effectors.drag.drag_coefficient == 2.2


# ---------------------------------------------------------------------------
# RecorderConfig tests
# ---------------------------------------------------------------------------

class TestRecorderConfig:
    """Tests for recorder configuration."""

    def test_default_recorder_enabled(self):
        """Recorder defaults to enabled with no explicit sample time."""
        cfg = RecorderConfig()
        assert cfg.enabled is True
        assert cfg.sample_time is None

    def test_recorder_with_explicit_sample_time(self):
        """Recorder with explicit sample time."""
        cfg = RecorderConfig(sample_time=0.5)
        assert cfg.enabled is True
        assert cfg.sample_time == 0.5

    def test_recorder_disabled(self):
        """Recorder can be explicitly disabled."""
        cfg = RecorderConfig(enabled=False)
        assert cfg.enabled is False

    def test_recorder_disabled_with_sample_time(self):
        """Disabled recorder ignores sample time in runner."""
        cfg = RecorderConfig(enabled=False, sample_time=0.1)
        assert cfg.enabled is False
        assert cfg.sample_time == 0.1


class TestRecorderOnSpacecraft:
    """Tests that recorder fields are present on configs."""

    def test_spacecraft_has_recorder(self):
        """SpacecraftConfig has recorder field."""
        sc = SpacecraftConfig(name="test")
        assert sc.recorder is not None
        assert sc.recorder.enabled is True

    def test_sensor_has_recorder(self):
        """Sensor configs have recorder field."""
        sun = SunSensorConfig()
        assert sun.recorder is not None
        assert sun.recorder.enabled is True

        mag = MagnetometerConfig()
        assert mag.recorder is not None

        st = StarTrackerConfig()
        assert st.recorder is not None

        imu = IMUConfig()
        assert imu.recorder is not None

        cam = CameraConfig()
        assert cam.recorder is not None

    def test_rw_set_has_recorder(self):
        """ReactionWheelSetConfig has recorder field."""
        rw_set = ReactionWheelSetConfig(
            wheels=[SingleReactionWheelConfig(name="RW-x")]
        )
        assert rw_set.recorder is not None
        assert rw_set.recorder.enabled is True

    def test_thruster_has_recorder(self):
        """ThrusterConfig has recorder field."""
        t = ThrusterConfig()
        assert t.recorder is not None
        assert t.recorder.enabled is True


class TestParserRecorderConfig:
    """Tests for TOML parsing of recorder settings."""

    @pytest.fixture
    def examples_dir(self) -> Path:
        """Path to the examples directory."""
        root = Path(__file__).resolve().parent.parent
        return root / "examples"

    def test_recorder_defaults_parsed(self, examples_dir):
        """Recorders default to enabled when not specified in TOML."""
        cfg = load_simulation_config(examples_dir / "single_sat_leo.toml")
        sat = cfg.satellites[0]

        # State recorder defaults
        assert sat.recorder.enabled is True
        assert sat.recorder.sample_time is None

        # Torque recorder defaults (if attitude control present)
        if sat.attitude and sat.attitude.control:
            assert sat.attitude.control.recorder.enabled is True
