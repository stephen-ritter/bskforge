#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Loader and schema tests (no Basilisk import required)."""

from pathlib import Path

import pytest

from bskforge.errors import ConfigError
from bskforge.loader import load_scenario, validate_scenario

EXAMPLES = Path(__file__).resolve().parent.parent / "examples"


def minimal_scenario(**overrides):
    data = {
        "simulation": {"name": "test", "duration": 100.0},
        "spacecraft": [{"name": "sc1"}],
    }
    data.update(overrides)
    return data


class TestValidation:
    def test_minimal_scenario_validates(self):
        config = validate_scenario(minimal_scenario())
        assert config.spacecraft[0].name == "sc1"
        assert config.world.central_body == "earth"

    def test_unknown_top_level_key_suggests(self):
        data = minimal_scenario(spacecraftX=[{"name": "a"}])
        with pytest.raises(ConfigError, match="unknown key 'spacecraftX'"):
            validate_scenario(data)

    def test_unknown_nested_key_suggests(self):
        data = minimal_scenario()
        data["simulation"]["duratioon"] = 10.0
        with pytest.raises(ConfigError, match="simulation"):
            validate_scenario(data)

    def test_inertia_table_normalizes_to_flat(self):
        data = minimal_scenario()
        data["spacecraft"][0]["hub"] = {
            "mass": 500.0,
            "inertia": {"ixx": 9.0, "iyy": 8.0, "izz": 7.0, "ixy": 1.0},
        }
        config = validate_scenario(data)
        assert config.spacecraft[0].hub.inertia == [
            9.0,
            1.0,
            0.0,
            1.0,
            8.0,
            0.0,
            0.0,
            0.0,
            7.0,
        ]

    def test_duplicate_spacecraft_names_rejected(self):
        data = minimal_scenario(spacecraft=[{"name": "sc1"}, {"name": "sc1"}])
        with pytest.raises(ConfigError, match="duplicate spacecraft"):
            validate_scenario(data)

    def test_allocation_requires_reaction_wheels(self):
        data = minimal_scenario()
        data["spacecraft"][0]["fsw"] = {"allocation": {"type": "rwMotorTorque"}}
        with pytest.raises(ConfigError, match="no reactionWheels effector"):
            validate_scenario(data)

    def test_orbit_union_discriminates(self):
        data = minimal_scenario()
        data["spacecraft"][0]["orbit"] = {
            "type": "rv",
            "rv": {"position": [7e6, 0, 0], "velocity": [0, 7.5, 0]},
        }
        config = validate_scenario(data)
        orbit = config.spacecraft[0].orbit
        assert orbit is not None
        assert orbit.type == "rv"

    def test_custom_gravity_body_requires_mu(self):
        data = minimal_scenario()
        data["world"] = {
            "central_body": "kepler-442",
            "gravities": [{"name": "kepler-442", "type": "custom", "central": True}],
        }
        with pytest.raises(ConfigError, match="mu"):
            validate_scenario(data)


class TestExtends:
    def test_child_overrides_base(self, tmp_path: Path):
        base = tmp_path / "base.toml"
        base.write_text('[simulation]\nname = "base"\nduration = 100.0\n')
        child = tmp_path / "child.toml"
        child.write_text(
            f'extends = "{base.name}"\n'
            "[simulation]\nduration = 50.0\n"
            '[[spacecraft]]\nname = "sc1"\n'
        )
        config = load_scenario(child)
        assert config.simulation.name == "base"
        assert config.simulation.duration == 50.0

    def test_missing_parent_errors(self, tmp_path: Path):
        child = tmp_path / "child.toml"
        child.write_text('extends = "nope.toml"\n')
        with pytest.raises(ConfigError, match="extended file not found"):
            load_scenario(child)

    def test_circular_extends_errors(self, tmp_path: Path):
        (tmp_path / "a.toml").write_text('extends = "b.toml"\n')
        (tmp_path / "b.toml").write_text('extends = "a.toml"\n')
        with pytest.raises(ConfigError, match="circular"):
            load_scenario(tmp_path / "a.toml")


@pytest.mark.parametrize(
    "example",
    sorted(EXAMPLES.glob("*.toml")),
    ids=lambda p: p.name,
)
def test_examples_validate(example: Path):
    config = load_scenario(example)
    assert config.spacecraft, f"{example.name} declares no spacecraft"
