#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""End-to-end build/run tests against the installed Basilisk build."""

import copy
import math
from pathlib import Path

import numpy as np
import pytest

from bskforge.errors import BuildError, ConfigError, UnsupportedError
from bskforge.loader import load_scenario, validate_scenario
from bskforge.montecarlo import run_monte_carlo
from bskforge.runner import ScenarioRunner

EXAMPLES = Path(__file__).resolve().parent.parent / "examples"


def minimal_scenario():
    return validate_scenario(
        {
            "simulation": {"name": "t", "duration": 10.0},
            "spacecraft": [{"name": "sc1"}],
        }
    )


class TestBuild:
    def test_minimal_scenario_builds(self):
        built = ScenarioRunner(minimal_scenario()).build()
        # hub + extFT registered
        assert "sc1" in built.context.modules_by_tag
        assert built.context.central_mu > 0

    def test_spice_without_epoch_fails_clearly(self):
        config = validate_scenario(
            {
                "world": {"spice": {"enabled": True}},
                "spacecraft": [{"name": "sc1"}],
            }
        )
        with pytest.raises(BuildError, match="epoch"):
            ScenarioRunner(config).build()

    def test_spice_world_builds(self):
        config = validate_scenario(
            {
                "world": {
                    "epoch": "2026-01-01 00:00:00.0 TDB",
                    "spice": {"enabled": True},
                    "gravities": [{"name": "moon"}],
                },
                "spacecraft": [{"name": "sc1"}],
            }
        )
        built = ScenarioRunner(config).build()
        assert built.context.spice is not None
        assert "earth" in built.context.planet_state_msgs
        assert "moon" in built.context.planet_state_msgs

    def test_spherical_harmonics_j2_builds(self):
        config = validate_scenario(
            {
                "world": {
                    "gravities": [
                        {
                            "name": "earth",
                            "central": True,
                            "grav_order": 2,
                            "grav_file": "GGM03S_J2_only",
                        }
                    ]
                },
                "spacecraft": [{"name": "sc1"}],
            }
        )
        built = ScenarioRunner(config).build()
        model_type = type(built.context.central_body.gravityModel).__name__
        assert model_type == "SphericalHarmonicsGravityModel"

    def test_magnetometer_requires_environment(self):
        config = validate_scenario(
            {"spacecraft": [{"name": "sc1", "sensors": [{"type": "magnetometer"}]}]}
        )
        with pytest.raises(BuildError, match="magnetic_field"):
            ScenarioRunner(config).build()

    def test_drag_requires_atmosphere(self):
        config = validate_scenario(
            {"spacecraft": [{"name": "sc1", "disturbances": [{"type": "drag"}]}]}
        )
        with pytest.raises(BuildError, match="atmosphere"):
            ScenarioRunner(config).build()

    def test_solar_flux_unsupported(self):
        config = validate_scenario(
            {
                "environment": {"solar_flux": 1361.0},
                "spacecraft": [{"name": "sc1"}],
            }
        )
        with pytest.raises(UnsupportedError, match="solar_flux"):
            ScenarioRunner(config).build()

    def test_magnetic_field_and_magnetometer_builds(self):
        config = validate_scenario(
            {
                "environment": {"magnetic_field": {"type": "centered_dipole"}},
                "spacecraft": [
                    {
                        "name": "sc1",
                        "sensors": [{"type": "magnetometer", "name": "tam"}],
                    }
                ],
            }
        )
        built = ScenarioRunner(config).build()
        assert ("sc1", "magnetic_field") in built.context.env_msgs
        assert "sc1_tam" in built.context.modules_by_tag

    def test_per_vehicle_environment_override(self):
        config = validate_scenario(
            {
                "environment": {"magnetic_field": {"type": "centered_dipole"}},
                "spacecraft": [
                    {"name": "chief"},
                    {
                        "name": "deputy",
                        "environment": {"magnetic_field": {"type": "wmm"}},
                    },
                ],
            }
        )
        built = ScenarioRunner(config).build()
        chief_env = built.context.bundles["chief"].resolved_env
        deputy_env = built.context.bundles["deputy"].resolved_env
        assert chief_env is not None and chief_env.magnetic_field.type == (
            "centered_dipole"
        )
        assert deputy_env is not None and deputy_env.magnetic_field.type == "wmm"

    def test_unknown_param_fails_at_build_with_hint(self):
        config = validate_scenario(
            {
                "spacecraft": [
                    {
                        "name": "sc1",
                        "sensors": [
                            {
                                "type": "coarseSunSensor",
                                "name": "css1",
                                "params": {"nHat_Bb": [1, 0, 0]},
                            }
                        ],
                    }
                ]
            }
        )
        with pytest.raises(Exception, match="nHat_B"):
            ScenarioRunner(config).build()

    def test_custom_module_builds_and_registers(self):
        config = validate_scenario(
            {
                "spacecraft": [{"name": "sc1"}],
                "custom": {
                    "modules": [
                        {
                            "module": ("Basilisk.simulation.simpleNav.SimpleNav"),
                            "tag": "nav1",
                        }
                    ]
                },
            }
        )
        built = ScenarioRunner(config).build()
        assert "nav1" in built.context.modules_by_tag

    def test_custom_connection_unknown_tag_errors(self):
        config = validate_scenario(
            {
                "spacecraft": [{"name": "sc1"}],
                "custom": {
                    "connections": [
                        {
                            "source": "nope.attOutMsg",
                            "destination": "sc1.dummyInMsg",
                        }
                    ]
                },
            }
        )
        with pytest.raises(Exception, match="unknown tag"):
            ScenarioRunner(config).build()


class TestRun:
    @pytest.mark.parametrize(
        "example,expected_streams",
        [
            (
                "single_sat_leo.toml",
                {"sc1.state", "sc1.control_torque", "sc1.rw_speeds"},
            ),
            (
                "multi_sat_formation.toml",
                {
                    "chief.state",
                    "chief.control_torque",
                    "deputy.state",
                    "deputy.rw_speeds",
                },
            ),
            (
                "single_sat_with_sensors.toml",
                {"sc1.state", "sc1.css1", "sc1.css2", "sc1.st", "sc1.imu"},
            ),
            ("custom_escape_hatch.toml", {"sc1.state"}),
            (
                "earth_environment.toml",
                {"sc1.state", "sc1.tam", "sc1.css1", "sc1.st", "sc1.imu"},
            ),
            (
                "estimation_and_opnav.toml",
                {
                    "nadir.state",
                    "nadir.nav_state",
                    "nadir.lmk",
                    "att.nav_state",
                    "att.control_torque",
                },
            ),
            (
                "momentum_management.toml",
                {
                    "thrusterSat.state",
                    "thrusterSat.rw_speeds",
                    "thrusterSat.thruster_cmds",
                    "mtbSat.mtb_cmds",
                },
            ),
            (
                "guidance_laws.toml",
                {"chaser.state", "tumor.state"},
            ),
            (
                "ground_pointing.toml",
                {
                    "imager.state",
                    "imager.control_torque",
                    "imager.rw_speeds",
                    "imager.st",
                },
            ),
            (
                "relay_chain.toml",
                {
                    "chain.barycenter",
                    "chain.rel_deputy",
                    "feed.link",
                    "down.link",
                    "down.access",
                },
            ),
            (
                "comm_links.toml",
                {"down.link", "down.access", "xlink.link", "relay.state"},
            ),
            (
                "monte_carlo.toml",
                {"sc1.state"},
            ),
            (
                "formation_flying.toml",
                {
                    "train.barycenter",
                    "train.rel_ahead",
                    "train.rel_below",
                    "chief.state",
                },
            ),
        ],
    )
    def test_examples_run_short_duration(self, example, expected_streams):
        config = load_scenario(EXAMPLES / example)
        result = ScenarioRunner(config).run(duration=30.0)
        assert expected_streams.issubset(result.frames)
        state_stream = next(k for k in sorted(result.frames) if k.endswith(".state"))
        assert len(result.frames[state_stream]) > 10

    def test_attitude_converges_with_rw_control(self):
        config = load_scenario(EXAMPLES / "single_sat_leo.toml")
        result = ScenarioRunner(config).run(duration=400.0)
        df = result.frames["sc1.state"]
        sigma = df[["sigma_BN[0]", "sigma_BN[1]", "sigma_BN[2]"]].iloc[-1].to_numpy()
        omega = (
            df[["omega_BN_B[0]", "omega_BN_B[1]", "omega_BN_B[2]"]].iloc[-1].to_numpy()
        )
        # proportional-only MRP feedback on wheels settles with a small
        # pointing error balanced by the wheels' absorbed momentum
        assert np.linalg.norm(sigma) < 2e-3, "attitude error did not converge"
        assert np.linalg.norm(omega) < 5e-4, "body rate did not converge"

    def test_sensor_values_physical(self):
        config = load_scenario(EXAMPLES / "single_sat_with_sensors.toml")
        result = ScenarioRunner(config).run(duration=60.0)
        # css1 boresight +X, sun fixed +X: illuminated late in run
        assert result.frames["sc1.css1"]["OutputData"].iloc[-1] > 0.5
        # css2 boresight -X: never illuminated
        assert result.frames["sc1.css2"]["OutputData"].max() == pytest.approx(0.0)

    def test_environment_disturbance_physics(self):
        """earth_environment.toml: fields, forces and ephemerides add up."""
        config = load_scenario(EXAMPLES / "earth_environment.toml")
        built = ScenarioRunner(config).build()
        ctx = built.context
        ctx.sim.ConfigureStopTime(int(120e9))
        ctx.sim.InitializeSimulation()
        ctx.sim.ExecuteSimulation()
        mods = ctx.modules_by_tag

        # WMM field at LEO is tens of microtesla
        tam = mods["sc1_tam"]
        field = np.linalg.norm(np.array(tam.tam_S))
        assert 1e-6 < field < 1e-3, f"magnetometer field {field}"

        # drag: density chain feeds the cannonball effector
        drag = mods["sc1_drag"]
        assert drag.getDensity() > 0
        assert np.linalg.norm(np.array(drag.forceExternal_B)) > 0

        # SRP: cannonball force from the SPICE sun line (this module keeps
        # its last force in the inertial frame array)
        srp = mods["sc1_srp"]
        assert srp.sunEphmInMsg.isWritten()
        illum = ctx.eclipse.eclipseOutMsgs[0].read().shadowFactor
        srp_force = max(
            np.linalg.norm(np.array(srp.forceExternal_N)),
            np.linalg.norm(np.array(srp.forceExternal_B)),
        )
        if illum > 0.5:  # sunlit sample (epoch is fixed -> deterministic)
            assert 1e-6 < srp_force < 1e-3, f"SRP force {srp_force}"

        # gravity gradient produces a (small) torque
        gg = mods["sc1_gravGrad"]
        assert np.linalg.norm(np.array(gg.torqueExternalPntB_B)) > 0

        # SPICE sun ephemeris near 1 au from Earth
        sun = np.linalg.norm(
            np.array(ctx.planet_state_msgs["sun"].read().PositionVector)
        )
        assert 1.4e11 < sun < 1.6e11

    def test_faceted_drag_and_srp_compute_forces(self):
        config = validate_scenario(
            {
                "environment": {
                    "atmosphere": {"type": "exponential"},
                    "eclipse": {"type": "penumbra"},
                },
                "spacecraft": [
                    {
                        "name": "sc1",
                        "orbit": {
                            "type": "elements",
                            "elements": {"a": 6671e3},
                        },
                        "disturbances": [
                            {
                                "type": "drag",
                                "model": "faceted",
                                "facets": [
                                    {"area": 5.0, "normal_B": [1, 0, 0]},
                                    {"area": 5.0, "normal_B": [-1, 0, 0]},
                                ],
                            },
                            {
                                "type": "solarRadiationPressure",
                                "model": "faceted",
                                "facets": [{"area": 10.0, "nHat_B": [1, 0, 0]}],
                            },
                        ],
                    }
                ],
            }
        )
        built = ScenarioRunner(config).build()
        ctx = built.context
        ctx.sim.ConfigureStopTime(int(60e9))
        ctx.sim.InitializeSimulation()
        ctx.sim.ExecuteSimulation()
        drag = ctx.modules_by_tag["sc1_drag"]
        assert drag.numFacets == 2
        atmo_msg = ctx.env_msgs[("sc1", "atmosphere")]
        assert atmo_msg.read().neutralDensity > 0
        srp = ctx.modules_by_tag["sc1_srp"]
        illum = ctx.eclipse.eclipseOutMsgs[0].read().shadowFactor
        force = max(
            np.linalg.norm(np.array(srp.forceExternal_N)),
            np.linalg.norm(np.array(srp.forceExternal_B)),
        )
        if illum > 0.5:
            assert force > 0

    def test_magnetometer_field_varies_in_orbit(self):
        config = load_scenario(EXAMPLES / "earth_environment.toml")
        result = ScenarioRunner(config).run(duration=600.0)
        df = result.frames["sc1.tam"]
        b = df[["tam_S[0]", "tam_S[1]", "tam_S[2]"]].to_numpy()
        norms = np.linalg.norm(b, axis=1)
        assert norms[-1] > 1e-6
        # field vector rotates in the body frame over the orbit
        spread = np.linalg.norm(b.max(axis=0) - b.min(axis=0))
        assert spread > 1e-5

    def test_export_writes_csv(self, tmp_path: Path):
        config = load_scenario(EXAMPLES / "single_sat_leo.toml")
        result = ScenarioRunner(config).run(duration=20.0, output_directory=tmp_path)
        files = sorted(p.name for p in tmp_path.glob("*.csv"))
        assert "sc1.state.csv" in files
        assert result.exported_files


def _css_cluster(name: str) -> dict:
    """Eight-pyramid CSS cluster sensitive to a sun line along +X."""
    boresights = [
        [0.7071, -0.5, 0.5],
        [0.7071, -0.5, -0.5],
        [0.7071, 0.5, -0.5],
        [0.7071, 0.5, 0.5],
        [-0.7071, 0.0, 0.7071],
        [-0.7071, 0.7071, 0.0],
        [-0.7071, 0.0, -0.7071],
        [-0.7071, -0.7071, 0.0],
    ]
    return {
        "name": name,
        "orbit": {"type": "elements", "elements": {"a": 7000e3}},
        "sensors": [
            {
                "type": "coarseSunSensor",
                "name": f"c{i}",
                "boresight_B": b,
                "fov_deg": 50,
            }
            for i, b in enumerate(boresights)
        ],
        "css_constellations": [
            {"name": "group", "members": [f"c{i}" for i in range(8)]}
        ],
    }


class TestEstimationAndCameras:
    def test_css_wls_estimator_recovers_sun_direction(self):
        config = validate_scenario(
            {
                "simulation": {"name": "wls", "duration": 60.0},
                "spacecraft": [
                    {
                        **_css_cluster("sc1"),
                        "fsw": {"estimator": {"type": "cssWlsEst", "cluster": "group"}},
                    }
                ],
            }
        )
        result = ScenarioRunner(config).run(duration=60.0)
        sun = result.frames["sc1.nav_state"][
            ["vehSunPntBdy[0]", "vehSunPntBdy[1]", "vehSunPntBdy[2]"]
        ].to_numpy()[-1]
        assert np.linalg.norm(sun) == pytest.approx(1.0, abs=1e-3)
        assert np.linalg.norm(sun - np.array([1.0, 0.0, 0.0])) < 0.1

    def test_inertial_ukf_converges_to_truth(self):
        config = validate_scenario(
            {
                "simulation": {"name": "ukf", "duration": 60.0},
                "spacecraft": [
                    {
                        "name": "sc1",
                        "orbit": {"type": "elements", "elements": {"a": 7200e3}},
                        "attitude": {
                            "initial": {
                                "sigma_BN": [0.1, -0.2, 0.05],
                                "omega_BN_B": [0.001, -0.01, 0.03],
                            }
                        },
                        "sensors": [
                            {"type": "starTracker", "name": "st1", "white_noise": 1e-3},
                            {"type": "starTracker", "name": "st2"},
                        ],
                        "fsw": {"estimator": {"type": "inertialUKF"}},
                    }
                ],
            }
        )
        result = ScenarioRunner(config).run(duration=60.0)
        cols = ["sigma_BN[0]", "sigma_BN[1]", "sigma_BN[2]"]
        est_frame = result.frames["sc1.nav_state"]
        truth_frame = result.frames["sc1.state"]
        est = est_frame[cols].to_numpy()
        t_est = est_frame["time"].to_numpy()
        t_truth = truth_frame["time"].to_numpy()
        truth = np.column_stack(
            [np.interp(t_est, t_truth, truth_frame[col].to_numpy()) for col in cols]
        )
        assert np.isfinite(est).all()
        assert np.linalg.norm(est[-1]) > 1e-2, "filter never left its zero init"
        # the residual floor is set by the ST white noise (1e-3/axis); the
        # mean over the settled quarter is the stable convergence measure
        tail = slice(len(est) // 4 * 3, None)
        gap = np.linalg.norm(est[tail] - truth[tail], axis=1)
        assert gap.mean() < 1.5e-3, (
            f"filter did not converge: mean gap {gap.mean():.1e}"
        )

    def test_inertial_ukf_requires_star_tracker(self):
        config = validate_scenario(
            {
                "spacecraft": [
                    {
                        "name": "sc1",
                        "fsw": {"estimator": {"type": "inertialUKF"}},
                    }
                ]
            }
        )
        with pytest.raises(BuildError, match="starTracker"):
            ScenarioRunner(config).build()

    def test_estimator_unknown_cluster_names_available(self):
        config = validate_scenario(
            {
                "spacecraft": [
                    {
                        **_css_cluster("sc1"),
                        "fsw": {"estimator": {"type": "cssWlsEst", "cluster": "nope"}},
                    }
                ]
            }
        )
        with pytest.raises(BuildError, match="css_constellation"):
            ScenarioRunner(config).build()

    def test_camera_observes_surface_landmark(self):
        config = validate_scenario(
            {
                "simulation": {"name": "opnav", "duration": 30.0},
                "world": {
                    "epoch": "2026-03-20 12:05:06.0 (UTC)",
                    "spice": {"enabled": True, "zero_base": "sun"},
                },
                "spacecraft": [
                    {
                        "name": "sc1",
                        "orbit": {"type": "elements", "elements": {"a": 7000e3}},
                        "sensors": [
                            {
                                "type": "camera",
                                "name": "cam",
                                "sigma_CB": [0.0, -0.41421356, 0.0],
                                "landmarks": [
                                    {
                                        "position": [6378e3, 0.0, 0.0],
                                        "normal": [1.0, 0.0, 0.0],
                                    }
                                ],
                            }
                        ],
                    }
                ],
            }
        )
        result = ScenarioRunner(config).run(duration=30.0)
        df = result.frames["sc1.cam"]
        visible = df["isVisible"].to_numpy() > 0
        assert visible.sum() >= 1, "landmark never entered the field of view"
        pixels = df.loc[visible, ["pL[0]", "pL[1]"]].to_numpy()
        assert (np.abs(pixels) <= 1024).all()

    def test_camera_requires_spice_target(self):
        config = validate_scenario(
            {
                "spacecraft": [
                    {
                        "name": "sc1",
                        "sensors": [
                            {
                                "type": "camera",
                                "name": "cam",
                                "landmarks": [{"position": [6378e3, 0.0, 0.0]}],
                            }
                        ],
                    }
                ]
            }
        )
        with pytest.raises(BuildError, match="SPICE"):
            ScenarioRunner(config).build()

    def test_star_tracker_white_noise_scatters_around_truth(self):
        config = validate_scenario(
            {
                "spacecraft": [
                    {
                        "name": "sc1",
                        "sensors": [
                            {"type": "starTracker", "name": "st", "white_noise": 0.01}
                        ],
                    }
                ]
            }
        )
        result = ScenarioRunner(config).run(duration=30.0)
        sensed = result.frames["sc1.st"][
            ["qInrtl2Case[0]", "qInrtl2Case[1]", "qInrtl2Case[2]", "qInrtl2Case[3]"]
        ].to_numpy()
        truth = result.frames["sc1.state"][
            ["sigma_BN[0]", "sigma_BN[1]", "sigma_BN[2]"]
        ].to_numpy()
        from Basilisk.utilities import RigidBodyKinematics as rbk

        truth_quat = np.array([rbk.MRP2EP(s) for s in truth[1:]])
        err = np.linalg.norm(sensed[1:] - truth_quat, axis=1)
        assert err.max() > 1e-6, "noise did not reach the sensor output"
        assert err.max() < 0.05, f"noise too large: {err.max()}"

    def test_imu_bias_shows_in_gyro_output(self):
        config = validate_scenario(
            {
                "spacecraft": [
                    {
                        "name": "sc1",
                        "attitude": {"initial": {"omega_BN_B": [0.01, 0.0, 0.0]}},
                        "sensors": [
                            {
                                "type": "imuSensor",
                                "name": "imu",
                                "gyro_bias": [0.002, 0.0, 0.0],
                            }
                        ],
                    }
                ]
            }
        )
        result = ScenarioRunner(config).run(duration=10.0)
        gyro = result.frames["sc1.imu"]["AngVelPlatform[0]"].to_numpy()
        assert gyro[-1] == pytest.approx(0.012, abs=2e-3)

    def test_css_cluster_member_must_be_a_css(self):
        with pytest.raises(ConfigError, match="unknown coarse sun sensors"):
            validate_scenario(
                {
                    "spacecraft": [
                        {
                            "name": "sc1",
                            "sensors": [
                                {"type": "coarseSunSensor", "name": "c1"},
                                {"type": "starTracker", "name": "st"},
                            ],
                            "css_constellations": [
                                {"name": "group", "members": ["c1", "st"]}
                            ],
                        }
                    ]
                }
            )


def _thruster_set(max_thrust: float = 5.0) -> list[dict]:
    """Standard 8-thruster cross-X configuration (fully actuated)."""
    location = [
        [0.5, 0.5, 0.3],
        [0.5, -0.5, 0.3],
        [0.5, 0.5, -0.3],
        [0.5, -0.5, -0.3],
        [-0.5, 0.5, 0.3],
        [-0.5, -0.5, 0.3],
        [-0.5, 0.5, -0.3],
        [-0.5, -0.5, -0.3],
    ]
    direction = [
        [1, 0, 0],
        [-1, 0, 0],
        [0, 1, 0],
        [0, -1, 0],
        [-1, 0, 0],
        [1, 0, 0],
        [0, -1, 0],
        [0, 1, 0],
    ]
    return [
        {
            "name": f"t{i}",
            "direction_B": d,
            "position_B": p,
            "max_thrust": max_thrust,
        }
        for i, (p, d) in enumerate(zip(location, direction, strict=True))
    ]


RW_SET = {
    "type": "reactionWheels",
    "inertia": 0.2,
    "wheels": [
        {"direction_B": [1, 0, 0], "initial_speed": 3000.0, "max_torque": 1.0},
        {"direction_B": [0, 1, 0], "initial_speed": -2500.0, "max_torque": 1.0},
        {"direction_B": [0, 0, 1], "initial_speed": 2000.0, "max_torque": 1.0},
    ],
}

ATT_HUB = {"mass": 750.0, "inertia": {"ixx": 900.0, "iyy": 800.0, "izz": 600.0}}


def _net_wheel_momentum(frame, js: float) -> np.ndarray:
    cols = [c for c in frame.columns if c.startswith("wheelSpeeds")]
    return js * frame[cols].to_numpy()


class TestThrustersAndMtb:
    def test_thrusters_stabilise_attitude(self):
        config = validate_scenario(
            {
                "simulation": {
                    "name": "rcs",
                    "duration": 300.0,
                    "dt": {"dynamics": 0.1, "fsw": 1.0},
                },
                "spacecraft": [
                    {
                        "name": "sc1",
                        "hub": ATT_HUB,
                        "orbit": {
                            "type": "elements",
                            "elements": {"a": 7000e3},
                        },
                        "attitude": {"initial": {"sigma_BN": [0.2, -0.1, 0.15]}},
                        "effectors": [
                            {
                                "type": "thrusters",
                                "thrusters": _thruster_set(20.0),
                            }
                        ],
                        "fsw": {
                            "guidance": {"type": "inertial3D", "sigma_R0N": [0, 0, 0]},
                            "control": {"type": "mrpFeedback", "k": 30.0, "p": 40.0},
                            "allocation": {
                                "type": "thrusterTorque",
                                "min_fire_time": 0.05,
                            },
                        },
                    }
                ],
            }
        )
        result = ScenarioRunner(config).run(duration=300.0)
        cols = ["sigma_BN[0]", "sigma_BN[1]", "sigma_BN[2]"]
        sigma = result.frames["sc1.state"][cols].to_numpy()
        assert np.linalg.norm(sigma[-1]) < 0.5 * np.linalg.norm(sigma[0])
        cmds = result.frames["sc1.thruster_cmds"]
        pulses = cmds[[c for c in cmds.columns if c.startswith("OnTime")]].to_numpy()
        assert (pulses > 1e-9).sum() > 0, "thrusters never fired"

    def test_thruster_momentum_dumping_reduces_wheel_speed(self):
        config = validate_scenario(
            {
                "simulation": {
                    "name": "dump",
                    "duration": 600.0,
                    "dt": {"dynamics": 0.1, "fsw": 1.0},
                },
                "spacecraft": [
                    {
                        "name": "sc1",
                        "hub": ATT_HUB,
                        "orbit": {
                            "type": "elements",
                            "elements": {"a": 7000e3},
                        },
                        "attitude": {"initial": {"sigma_BN": [0.05, -0.05, 0.02]}},
                        "effectors": [
                            RW_SET,
                            {"type": "thrusters", "thrusters": _thruster_set()},
                        ],
                        "fsw": {
                            "estimator": {"type": "simpleNav"},
                            "guidance": {"type": "inertial3D", "sigma_R0N": [0, 0, 0]},
                            "control": {
                                "type": "mrpFeedback",
                                "k": 3.5,
                                "p": 30.0,
                                "ki": -1.0,
                            },
                            "allocation": {"type": "rwMotorTorque"},
                            "momentum_management": {
                                "type": "thrMomentumDumping",
                                "hs_min": 10.0,
                                "max_counter_value": 20,
                                "rearm_period": 10.0,
                            },
                        },
                    }
                ],
            }
        )
        result = ScenarioRunner(config).run(duration=600.0)
        momentum = _net_wheel_momentum(result.frames["sc1.rw_speeds"], js=0.2)
        start, end = np.linalg.norm(momentum[0]), np.linalg.norm(momentum[-1])
        assert end < 0.25 * start, f"momentum {start:.1f} -> {end:.1f} Nms"
        cols = ["sigma_BN[0]", "sigma_BN[1]", "sigma_BN[2]"]
        assert np.linalg.norm(result.frames["sc1.state"][cols].to_numpy()[-1]) < 0.05

    def test_mtb_momentum_management_desaturates(self):
        config = validate_scenario(
            {
                "simulation": {
                    "name": "mtb",
                    "duration": 1800.0,
                    "dt": {"dynamics": 0.5, "fsw": 1.0},
                },
                "environment": {"magnetic_field": {"type": "centered_dipole"}},
                "spacecraft": [
                    {
                        "name": "sc1",
                        "hub": {
                            "mass": 100.0,
                            "inertia": {"ixx": 100.0, "iyy": 90.0, "izz": 80.0},
                        },
                        "orbit": {
                            "type": "elements",
                            "elements": {"a": 6871e3},
                        },
                        "attitude": {"initial": {"sigma_BN": [0.02, -0.02, 0.01]}},
                        "effectors": [
                            {
                                "type": "reactionWheels",
                                "inertia": 0.002,
                                "wheels": [
                                    {
                                        "direction_B": [1, 0, 0],
                                        "initial_speed": 3000.0,
                                        "max_torque": 0.01,
                                    },
                                    {
                                        "direction_B": [0, 1, 0],
                                        "initial_speed": 2500.0,
                                        "max_torque": 0.01,
                                    },
                                    {
                                        "direction_B": [0, 0, 1],
                                        "initial_speed": -2000.0,
                                        "max_torque": 0.01,
                                    },
                                ],
                            },
                            {
                                "type": "mtb",
                                "bars": [
                                    {
                                        "direction_B": [0.577, 0.577, 0.577],
                                        "max_dipole": 50.0,
                                    },
                                    {
                                        "direction_B": [-0.577, 0.577, -0.577],
                                        "max_dipole": 50.0,
                                    },
                                    {
                                        "direction_B": [-0.577, -0.577, 0.577],
                                        "max_dipole": 50.0,
                                    },
                                    {
                                        "direction_B": [0.577, -0.577, -0.577],
                                        "max_dipole": 50.0,
                                    },
                                ],
                            },
                        ],
                        "fsw": {
                            "estimator": {"type": "simpleNav"},
                            "guidance": {"type": "inertial3D", "sigma_R0N": [0, 0, 0]},
                            "control": {
                                "type": "mrpFeedback",
                                "k": 3.5,
                                "p": 30.0,
                                "ki": -1.0,
                            },
                            "allocation": {"type": "rwMotorTorque"},
                            "momentum_management": {
                                "type": "mtbMomentumManagementSimple",
                                "kp": 0.003,
                            },
                        },
                    }
                ],
            }
        )
        result = ScenarioRunner(config).run(duration=1800.0)
        momentum = _net_wheel_momentum(result.frames["sc1.rw_speeds"], js=0.002)
        start, end = np.linalg.norm(momentum[0]), np.linalg.norm(momentum[-1])
        assert end < 0.6 * start, f"momentum {start:.2f} -> {end:.2f} Nms"
        cmds = result.frames["sc1.mtb_cmds"]
        dipoles = cmds[[c for c in cmds.columns if "ipole" in c]].to_numpy()
        assert (np.abs(dipoles) > 1e-9).sum() > 0

    def test_rw_null_space_steers_wheel_speeds(self):
        config = validate_scenario(
            {
                "simulation": {
                    "name": "nsp",
                    "duration": 900.0,
                    "dt": {"dynamics": 0.5, "fsw": 1.0},
                },
                "spacecraft": [
                    {
                        "name": "sc1",
                        "hub": {
                            "mass": 100.0,
                            "inertia": {"ixx": 100.0, "iyy": 90.0, "izz": 80.0},
                        },
                        "orbit": {
                            "type": "elements",
                            "elements": {"a": 6871e3},
                        },
                        "attitude": {"initial": {"sigma_BN": [0.02, -0.02, 0.01]}},
                        "effectors": [
                            {
                                "type": "reactionWheels",
                                "inertia": 0.05,
                                "wheels": [
                                    {
                                        "direction_B": [0.577, 0.577, 0.577],
                                        "initial_speed": 2000.0,
                                        "max_torque": 0.01,
                                    },
                                    {
                                        "direction_B": [-0.577, 0.577, -0.577],
                                        "initial_speed": 1800.0,
                                        "max_torque": 0.01,
                                    },
                                    {
                                        "direction_B": [-0.577, -0.577, 0.577],
                                        "initial_speed": -1500.0,
                                        "max_torque": 0.01,
                                    },
                                    {
                                        "direction_B": [0.577, -0.577, -0.577],
                                        "initial_speed": 1000.0,
                                        "max_torque": 0.01,
                                    },
                                ],
                            }
                        ],
                        "fsw": {
                            "estimator": {"type": "simpleNav"},
                            "guidance": {"type": "inertial3D", "sigma_R0N": [0, 0, 0]},
                            "control": {
                                "type": "mrpFeedback",
                                "k": 3.5,
                                "p": 30.0,
                                "ki": -1.0,
                            },
                            "allocation": {"type": "rwMotorTorque"},
                            "desaturation": {
                                "type": "rwNullSpace",
                                "omega_gain": 1e-4,
                            },
                        },
                    }
                ],
            }
        )
        result = ScenarioRunner(config).run(duration=900.0)
        speeds = _net_wheel_momentum(result.frames["sc1.rw_speeds"], js=0.05)
        assert np.abs(speeds[-1]).mean() < np.abs(speeds[0]).mean()
        cols = ["sigma_BN[0]", "sigma_BN[1]", "sigma_BN[2]"]
        assert np.linalg.norm(result.frames["sc1.state"][cols].to_numpy()[-1]) < 1e-3

    def test_thruster_allocation_requires_thrusters(self):
        with pytest.raises(ConfigError, match="thrusters effector"):
            validate_scenario(
                {
                    "spacecraft": [
                        {
                            "name": "sc1",
                            "effectors": [RW_SET],
                            "fsw": {
                                "control": {"type": "mrpFeedback"},
                                "allocation": {"type": "thrusterTorque"},
                            },
                        }
                    ]
                }
            )

    def test_momentum_management_requires_both_effectors(self):
        with pytest.raises(ConfigError, match="reactionWheels and thrusters"):
            validate_scenario(
                {
                    "spacecraft": [
                        {
                            "name": "sc1",
                            "effectors": [RW_SET],
                            "fsw": {
                                "momentum_management": {"type": "thrMomentumDumping"}
                            },
                        }
                    ]
                }
            )

    def test_dumping_conflicts_with_thruster_allocation(self):
        config = validate_scenario(
            {
                "spacecraft": [
                    {
                        "name": "sc1",
                        "effectors": [
                            RW_SET,
                            {"type": "thrusters", "thrusters": _thruster_set()},
                        ],
                        "fsw": {
                            "control": {"type": "mrpFeedback"},
                            "allocation": {"type": "thrusterTorque"},
                            "momentum_management": {"type": "thrMomentumDumping"},
                        },
                    }
                ]
            }
        )
        with pytest.raises(BuildError, match="both drive"):
            ScenarioRunner(config).build()

    def test_mtb_requires_magnetic_field(self):
        config = validate_scenario(
            {
                "spacecraft": [
                    {
                        "name": "sc1",
                        "effectors": [
                            {"type": "mtb", "bars": [{"direction_B": [1, 0, 0]}]}
                        ],
                    }
                ]
            }
        )
        with pytest.raises(BuildError, match="magnetic_field"):
            ScenarioRunner(config).build()


# -- Phase 5: guidance laws and control laws -----------------------------

SIGMA = ["sigma_BN[0]", "sigma_BN[1]", "sigma_BN[2]"]
OMEGA = ["omega_BN_B[0]", "omega_BN_B[1]", "omega_BN_B[2]"]

SPICE_WORLD = {
    "central_body": "earth",
    "epoch": "2026-01-01 00:00:00.0 TDB",
    "spice": {"enabled": True},
    "gravities": [
        {"name": "earth", "central": True},
        {"name": "sun"},
        {"name": "moon"},
    ],
}

_AGILE_HUB = {"mass": 100.0, "inertia": {"ixx": 100.0, "iyy": 90.0, "izz": 80.0}}
_AGILE_RW = {
    "type": "reactionWheels",
    "inertia": 0.05,
    "wheels": [
        {"direction_B": [1, 0, 0], "max_torque": 0.5},
        {"direction_B": [0, 1, 0], "max_torque": 0.5},
        {"direction_B": [0, 0, 1], "max_torque": 0.5},
    ],
}


def _guidance_config(
    guidance: dict,
    control: dict | None = None,
    *,
    world: dict | None = None,
    duration: float = 300.0,
    name: str = "sc1",
) -> dict:
    """A single agile SC on the 700 km circular equatorial orbit."""
    spacecraft: dict = {
        "name": name,
        "hub": dict(_AGILE_HUB),
        "orbit": {
            "type": "elements",
            "elements": {"a": 7200e3, "e": 0.0, "i": 0.0, "raan": 0.0},
        },
        "attitude": {
            "initial": {
                "sigma_BN": [0.1, -0.2, 0.05],
                "omega_BN_B": [0.001, -0.01, 0.03],
            },
        },
        "fsw": {
            "guidance": guidance,
            "control": control if control is not None else {"type": "mrpPD"},
        },
        "effectors": [copy.deepcopy(_AGILE_RW)],
    }
    scenario: dict = {
        "simulation": {"name": f"guid-{guidance['type']}", "duration": duration},
        "spacecraft": [spacecraft],
    }
    if world is not None:
        scenario["world"] = dict(world)
    return scenario


class TestGuidanceAndControl:
    """Each guidance law flies a closed loop; each control law converges."""

    def _tracking_metrics(
        self,
        guidance: dict,
        control: dict,
        duration: float,
    ) -> tuple[float, float]:
        """Final |sigma_BR| (tracking error) and |omega| of the run."""
        config = validate_scenario(
            _guidance_config(guidance, control, world=SPICE_WORLD, duration=duration)
        )
        built = ScenarioRunner(config).build()
        ctx = built.context
        err = ctx.modules_by_tag["sc1_attErr"]
        recorder = err.attGuidOutMsg.recorder(int(1e9))
        ctx.sim.AddModelToTask("fswTask", recorder, ModelPriority=1)
        ctx.sim.ConfigureStopTime(int(duration * 1e9))
        ctx.sim.InitializeSimulation()
        ctx.sim.ExecuteSimulation()
        sigma_br = np.array(recorder.sigma_BR)[-1]
        state = np.array(
            [float(x) for x in ctx.bundles["sc1"].sc.scStateOutMsg.read().omega_BN_B]
        )
        return float(np.linalg.norm(sigma_br)), float(np.linalg.norm(state))

    def test_hill_point_holds_lvlh(self):
        err_norm, omega = self._tracking_metrics(
            {"type": "hillPoint"}, {"type": "mrpPD"}, 300.0
        )
        assert err_norm < 1e-3
        # LVLH is a rotating frame: the body spins at the orbital rate
        n = math.sqrt(3.986004418e14 / 7200e3**3)
        assert omega == pytest.approx(n, rel=0.05)

    def test_location_point_tracks_ground_site(self):
        # SPICE planet: the +X-pointed loop must chase the site fixed
        # on the rotating Earth, keeping the tracking error bounded
        err_norm, omega = self._tracking_metrics(
            {"type": "locationPointing", "target_latlon": [0.0, 0.0, 0.0]},
            {"type": "mrpPD"},
            300.0,
        )
        assert err_norm < 0.05
        assert np.isfinite(omega)

    def test_velocity_point_tracks_velocity(self):
        err_norm, omega = self._tracking_metrics(
            {"type": "velocityPoint"}, {"type": "rateServo", "max_rate": 0.05}, 300.0
        )
        assert err_norm < 1e-3
        n = math.sqrt(3.986004418e14 / 7200e3**3)
        assert omega == pytest.approx(n, rel=0.05)

    def test_sun_safe_point_aligns_body_axis(self):
        # plain SimpleNav world: vehSunPntBdy = +X, so the loop must
        # simply damp out -- the axis already points at the sun
        config = validate_scenario(
            _guidance_config(
                {"type": "sunSafePoint", "sun_axis_B": [1.0, 0.0, 0.0]},
                world=None,
                duration=120.0,
            )
        )
        result = ScenarioRunner(config).run(duration=120.0)
        frame = result.frames["sc1.state"]
        assert np.isfinite(frame.to_numpy()).all()
        assert np.linalg.norm(frame[OMEGA].to_numpy()[-1]) < 0.02

    def test_waypoint_reference_follows_file(self, tmp_path):
        waypoints = tmp_path / "waypoints.csv"
        zero = "0.0,0.0,0.0"
        rows = [
            f"{t},{satt},{zero},{zero}"
            for t, satt in (
                (0.0, "0.0,0.0,0.0"),
                (300.0, "0.1,-0.1,0.05"),
                (600.0, "0.05,-0.2,0.0"),
            )
        ]
        waypoints.write_text(
            "time,s0,s1,s2,w0,w1,w2,dw0,dw1,dw2\n" + "\n".join(rows) + "\n",
            newline="",
        )
        config = validate_scenario(
            _guidance_config(
                {
                    "type": "waypointReference",
                    "waypoint_file": str(waypoints),
                    "header_lines": 1,
                },
                world=None,
                duration=620.0,
            )
        )
        # record the reference message itself: the file row must pass
        # through the module untouched once the final waypoint is active
        built = ScenarioRunner(config).build()
        ctx = built.context
        recorder = ctx.modules_by_tag["sc1_waypointReference"].attRefOutMsg.recorder(
            1_000_000_000
        )
        ctx.sim.AddModelToTask("fswTask", recorder, ModelPriority=1)
        ctx.sim.ConfigureStopTime(620_000_000_000)
        ctx.sim.InitializeSimulation()
        ctx.sim.ExecuteSimulation()
        sigma_rn = np.array(recorder.sigma_RN)[-1]
        assert np.allclose(sigma_rn, [0.05, -0.2, 0.0], atol=1e-9)

    def test_location_pointing_holds_ground_site(self):
        config = validate_scenario(
            _guidance_config(
                {"type": "locationPointing", "target_latlon": [0.0, 0.0, 0.0]},
                world=SPICE_WORLD,
                duration=600.0,
            )
        )
        result = ScenarioRunner(config).run(duration=600.0)
        frame = result.frames["sc1.state"]
        assert np.isfinite(frame.to_numpy()).all()
        # the closed loop tracks a rotating target (Earth rotation), so
        # the steady body rate is non-zero but small and the attitude is
        # stable, not tumbling
        assert np.linalg.norm(frame[OMEGA].to_numpy()[-1]) < 0.05

    def test_two_body_point_moon_sun(self):
        config = validate_scenario(
            _guidance_config(
                {
                    "type": "celestialTwoBodyPoint",
                    "target_body": "moon",
                    "constraint_body": "sun",
                },
                world=SPICE_WORLD,
                duration=120.0,
            )
        )
        result = ScenarioRunner(config).run(duration=120.0)
        frame = result.frames["sc1.state"]
        assert np.isfinite(frame.to_numpy()).all()
        assert np.linalg.norm(frame[OMEGA].to_numpy()[-1]) < 0.02

    def test_mrp_pd_and_rate_servo_build_same_reference(self):
        # the two control laws must consume the same reference chain
        for control in ({"type": "mrpPD"}, {"type": "rateServo"}):
            err_norm, _ = self._tracking_metrics({"type": "hillPoint"}, control, 180.0)
            assert err_norm < 5e-3, control["type"]

    def test_location_pointing_rejects_two_targets(self):
        with pytest.raises(ConfigError, match="exactly one of"):
            validate_scenario(
                _guidance_config(
                    {
                        "type": "locationPointing",
                        "target_latlon": [0.0, 0.0, 0.0],
                        "target_body": "moon",
                    },
                    world=SPICE_WORLD,
                )
            )

    def test_two_body_point_requires_spice(self):
        with pytest.raises(ConfigError, match="spice"):
            validate_scenario(
                _guidance_config(
                    {"type": "celestialTwoBodyPoint", "target_body": "moon"}
                )
            )

    def test_location_pointing_latlon_requires_spice(self):
        with pytest.raises(ConfigError, match="spice"):
            validate_scenario(
                _guidance_config(
                    {"type": "locationPointing", "target_latlon": [1.0, 2.0, 0.0]}
                )
            )


# -- Phase 6: comm links ----------------------------------------------------


def _comm_config(links: list[dict], *, world: dict | None = None) -> dict:
    """Two equatorial SC 1 degree of RAAN apart with comm links."""

    def sc(name: str, raan: float) -> dict:
        return {
            "name": name,
            "orbit": {
                "type": "elements",
                "elements": {"a": 7000e3, "e": 0.0, "i": 0.0, "raan": raan},
            },
        }

    scenario = {
        "simulation": {"name": "comm", "duration": 300.0},
        "spacecraft": [sc("a", 0.0), sc("b", 1.0)],
        "comm_links": links,
    }
    if world is not None:
        scenario["world"] = dict(world)
    return scenario


class TestCommLinks:
    """RF link chains: antennas, ground station and the link budget."""

    def test_crosslink_range_grows(self):
        config = validate_scenario(
            _comm_config(
                [
                    {
                        "name": "x",
                        "type": "crosslink",
                        "transmitter": "a",
                        "receiver": "b",
                    }
                ]
            )
        )
        result = ScenarioRunner(config).run(duration=300.0)
        link = result.frames["x.link"]
        dist = link["distance"].to_numpy()
        assert np.isfinite(dist).all()
        # a 1 deg RAAN separation at 7000 km opens at roughly n * R * dt
        assert dist[1] < dist[-1]
        assert dist[-1] == pytest.approx(7000e3 * math.radians(1.0), rel=0.25)
        # both ends hold the scheduled transmit/receive states
        assert set(link["antennaState1"].to_numpy()[1:]) == {2.0}
        assert set(link["antennaState2"].to_numpy()[1:]) == {1.0}
        assert np.all(link["CNR2"].to_numpy() >= 0.0)

    def test_downlink_geometry_and_access(self):
        config = validate_scenario(
            _comm_config(
                [
                    {
                        "name": "gs",
                        "type": "downlink",
                        "transmitter": "a",
                        "ground_station": {
                            "lat": 40.0069,
                            "long": -105.2649,
                            "elevation": 1655.0,
                        },
                    }
                ],
                world=SPICE_WORLD,
            )
        )
        result = ScenarioRunner(config).run(duration=300.0)
        link = result.frames["gs.link"]
        dist = link["distance"].to_numpy()[1:]
        assert np.isfinite(dist).all()
        assert np.all(dist > 0.0)
        # the slant range stays inside the chord through the Earth shell
        assert np.all(dist < 2.0 * 7000e3)
        access = result.frames["gs.access"]
        assert set(access["hasAccess"].to_numpy()).issubset({0.0, 1.0})
        rx = result.frames["gs.rx"]
        assert np.allclose(rx["frequency"].to_numpy()[1:], 8.4e9)

    def test_downlink_requires_spice(self):
        with pytest.raises(ConfigError, match="spice"):
            validate_scenario(
                _comm_config(
                    [
                        {
                            "name": "gs",
                            "type": "downlink",
                            "transmitter": "a",
                            "ground_station": {"lat": 0.0, "long": 0.0},
                        }
                    ]
                )
            )

    def test_comm_link_rejects_unknown_spacecraft(self):
        with pytest.raises(ConfigError, match="is not a"):
            validate_scenario(
                _comm_config(
                    [
                        {
                            "name": "x",
                            "type": "crosslink",
                            "transmitter": "a",
                            "receiver": "nobody",
                        }
                    ]
                )
            )

    def test_comm_link_rejects_self_loop(self):
        with pytest.raises(ConfigError, match="must differ"):
            validate_scenario(
                _comm_config(
                    [
                        {
                            "name": "x",
                            "type": "crosslink",
                            "transmitter": "a",
                            "receiver": "a",
                        }
                    ]
                )
            )


# -- Phase 6: formation flying ----------------------------------------------


def _formation_config(spacecraft: list[dict], formation: dict) -> dict:
    scenario = {
        "simulation": {"name": "formation", "duration": 300.0},
        "spacecraft": spacecraft,
        "formations": [formation],
    }
    return scenario


def _sc(name: str, raan: float, a: float = 7000e3, mass: float | None = None) -> dict:
    hub = {"mass": mass} if mass is not None else {}
    return {
        "name": name,
        "hub": hub,
        "orbit": {
            "type": "elements",
            "elements": {"a": a, "e": 0.0, "i": 0.0, "raan": raan},
        },
    }


R_COLS = ["r_BN_N[0]", "r_BN_N[1]", "r_BN_N[2]"]


class TestFormationFlying:
    """Barycenter averaging and Hill-frame relative states."""

    def test_hill_relative_state_is_steady_separation(self):
        config = validate_scenario(
            _formation_config(
                [_sc("chief", 0.0), _sc("dep", 0.5)],
                {"name": "club", "members": ["chief", "dep"]},
            )
        )
        result = ScenarioRunner(config).run(duration=300.0)
        rel = result.frames["club.rel_dep"]
        r = rel[["r_DC_H[0]", "r_DC_H[1]", "r_DC_H[2]"]].to_numpy()
        v = rel[["v_DC_H[0]", "v_DC_H[1]", "v_DC_H[2]"]].to_numpy()
        assert np.isfinite(r[1:]).all()
        # co-orbiting with a pure along-track offset: constant separation
        # along +y (the larger RAAN sits ahead on the equatorial circle),
        # radial and cross-track gaps stay shut
        y = r[-1, 1]
        assert y == pytest.approx(2 * 7000e3 * math.sin(math.radians(0.25)), rel=0.02)
        assert np.linalg.norm(r[-1, [0, 2]]) < 1e3
        assert np.linalg.norm(v[-1]) < 1.0

    def test_barycenter_is_mass_weighted(self):
        config = validate_scenario(
            _formation_config(
                [_sc("a", 0.0, mass=100.0), _sc("b", 0.5, mass=300.0)],
                {"name": "club", "members": ["a", "b"]},
            )
        )
        result = ScenarioRunner(config).run(duration=300.0)
        bar = result.frames["club.barycenter"][R_COLS].to_numpy()
        ra = result.frames["a.state"][R_COLS].to_numpy()
        rb = result.frames["b.state"][R_COLS].to_numpy()
        expected = (100.0 * ra + 300.0 * rb) / 400.0
        assert np.linalg.norm(bar[-1] - expected[-1]) < 2e3
        assert np.isfinite(bar[1:]).all()

    def test_barycenter_can_be_disabled(self):
        config = validate_scenario(
            _formation_config(
                [_sc("chief", 0.0), _sc("dep", 0.5)],
                {"name": "club", "members": ["chief", "dep"], "barycenter": False},
            )
        )
        result = ScenarioRunner(config).run(duration=60.0)
        assert "club.barycenter" not in result.frames
        assert "club.rel_dep" in result.frames

    def test_formation_rejects_unknown_member(self):
        with pytest.raises(ConfigError, match="are not"):
            validate_scenario(
                _formation_config(
                    [_sc("chief", 0.0)],
                    {"name": "club", "members": ["chief", "ghost"]},
                )
            )

    def test_formation_rejects_foreign_chief(self):
        with pytest.raises(ConfigError, match="not a member"):
            validate_scenario(
                _formation_config(
                    [_sc("chief", 0.0), _sc("dep", 0.5)],
                    {
                        "name": "club",
                        "members": ["chief", "dep"],
                        "relative_to": "stranger",
                    },
                )
            )


# -- Phase 6: Monte Carlo ----------------------------------------------------


class TestMonteCarlo:
    """Dispersion campaigns through Basilisk's MonteCarlo.Controller."""

    def _campaign_config(self, tmp_path, runs: int = 3) -> dict:
        return {
            "simulation": {"name": "mc-test", "duration": 60.0},
            "spacecraft": [
                {
                    "name": "sc1",
                    "orbit": {
                        "type": "elements",
                        "elements": {"a": 7200e3, "e": 0.001},
                    },
                    "attitude": {"initial": {"sigma_BN": [0.05, -0.05, 0.02]}},
                }
            ],
            "monte_carlo": {
                "runs": runs,
                "archive_dir": str(tmp_path / f"camp{runs}"),
                "dispersions": [
                    {
                        "spacecraft": "sc1",
                        "quantity": "orbit_elements",
                        "elements": {"a": ["normal", 7200e3, 50e3]},
                    },
                    {"spacecraft": "sc1", "quantity": "sigma_BN"},
                    {"spacecraft": "sc1", "quantity": "mass", "range": [95.0, 105.0]},
                ],
            },
        }

    def test_campaign_variates_the_state(self, tmp_path):
        config = validate_scenario(self._campaign_config(tmp_path))
        result = run_monte_carlo(config)
        assert result.failures == []
        assert len(result.summary) == 3
        a = result.summary["sc1.a"].to_numpy()
        # three runs, three different orbits, all within 3 sigma of nominal
        assert len(set(a)) == 3
        assert np.all(np.abs(a - 7200e3) < 2e5)
        sigma = result.summary["sc1.sigma_final"].to_numpy()
        assert len(set(np.round(sigma, 9))) == 3
        assert (tmp_path / "camp3").exists()

    def test_campaign_is_repeatable(self, tmp_path):
        result_a = run_monte_carlo(validate_scenario(self._campaign_config(tmp_path)))
        cfg2 = self._campaign_config(tmp_path)
        cfg2["monte_carlo"]["archive_dir"] = str(tmp_path / "again")
        result_b = run_monte_carlo(validate_scenario(cfg2))
        assert np.allclose(
            result_a.summary["sc1.a"].to_numpy(),
            result_b.summary["sc1.a"].to_numpy(),
        )

    def test_monte_carlo_rejects_unknown_spacecraft(self):
        with pytest.raises(ConfigError, match="not a"):
            validate_scenario(
                {
                    "simulation": {"name": "mc", "duration": 10.0},
                    "spacecraft": [{"name": "sc1"}],
                    "monte_carlo": {
                        "runs": 2,
                        "dispersions": [
                            {"spacecraft": "ghost", "quantity": "sigma_BN"}
                        ],
                    },
                }
            )
