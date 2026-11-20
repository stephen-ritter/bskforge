#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""End-to-end build/run tests against the installed Basilisk build."""

from pathlib import Path

import numpy as np
import pytest

from bskforge.errors import BuildError, UnsupportedError
from bskforge.loader import load_scenario, validate_scenario
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
        ],
    )
    def test_examples_run_short_duration(self, example, expected_streams):
        config = load_scenario(EXAMPLES / example)
        result = ScenarioRunner(config).run(duration=30.0)
        assert expected_streams.issubset(result.frames)
        assert (
            len(
                result.frames[
                    "sc1.state" if "sc1.state" in result.frames else "chief.state"
                ]
            )
            > 10
        )

    def test_attitude_converges_with_rw_control(self):
        config = load_scenario(EXAMPLES / "single_sat_leo.toml")
        result = ScenarioRunner(config).run(duration=400.0)
        df = result.frames["sc1.state"]
        sigma = df[["sigma_BN[0]", "sigma_BN[1]", "sigma_BN[2]"]].iloc[-1].to_numpy()
        omega = (
            df[["omega_BN_B[0]", "omega_BN_B[1]", "omega_BN_B[2]"]].iloc[-1].to_numpy()
        )
        assert np.linalg.norm(sigma) < 1e-4, "attitude error did not converge"
        assert np.linalg.norm(omega) < 1e-4, "body rate did not converge"

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
