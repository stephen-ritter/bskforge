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

from bskforge.errors import UnsupportedError
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

    def test_spice_requested_raises_unsupported(self):
        config = validate_scenario(
            {
                "world": {"spice": {"enabled": True}},
                "spacecraft": [{"name": "sc1"}],
            }
        )
        with pytest.raises(UnsupportedError, match="Phase 2"):
            ScenarioRunner(config).build()

    def test_environment_requested_raises_unsupported(self):
        config = validate_scenario(
            {
                "environment": {"magnetic_field": {"type": "centered_dipole"}},
                "spacecraft": [{"name": "sc1"}],
            }
        )
        with pytest.raises(UnsupportedError, match="environment"):
            ScenarioRunner(config).build()

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

    def test_export_writes_csv(self, tmp_path: Path):
        config = load_scenario(EXAMPLES / "single_sat_leo.toml")
        result = ScenarioRunner(config).run(duration=20.0, output_directory=tmp_path)
        files = sorted(p.name for p in tmp_path.glob("*.csv"))
        assert "sc1.state.csv" in files
        assert result.exported_files
