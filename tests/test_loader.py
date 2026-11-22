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

    def test_faceted_drag_requires_facets(self):
        data = minimal_scenario()
        data["spacecraft"][0]["disturbances"] = [{"type": "drag", "model": "faceted"}]
        with pytest.raises(ConfigError, match="requires facets"):
            validate_scenario(data)

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


class TestGeneratedReference:
    """bskforge reference: schema-derived field documentation."""

    def test_markdown_covers_sections_and_unions(self):
        from bskforge.reference import generate_markdown

        md = generate_markdown()
        assert "# `bskforge` scenario format reference" in md
        for heading in (
            "## `[simulation]`",
            "## `[[spacecraft]]`",
            "## `[[comm_links]]` — `downlink`",
            "## `[[formations]]`",
            "## `[monte_carlo]`",
            "### `[[spacecraft.effectors]]` — `reactionWheels`",
        ):
            assert heading in md, heading
        # discriminated unions list their kinds, defaults are shown
        assert "velocityPoint" in md
        assert "orbit_elements" in md
        assert "gpsReceiver" in md
        assert "`600.0`" in md  # simulation.duration default
        # tables shared by union variants are hoisted once
        assert md.count("### `[[spacecraft.sensors]].recorder`") == 1
        assert md.count("### `[[spacecraft.sensors]]` — ") == 6

    def test_toml_template_lists_fields_with_defaults(self):
        import tomllib

        from bskforge.reference import generate_toml_template

        template = generate_toml_template()
        # every content line is a comment: the template itself is valid TOML
        assert all(
            line.lstrip().startswith("#") or not line.strip()
            for line in template.splitlines()
        )
        tomllib.loads(template)
        for needle in (
            "# name = 'bskforge_scenario'",
            "# duration = 600.0",
            "# -- [[spacecraft.effectors]] --",
            "one entry — reactionWheels",
        ):
            assert needle in template, needle


class TestValidateCli:
    """bskforge validate: summary output and did-you-mean hints."""

    def _write(self, tmp_path, extra: str = "") -> Path:
        text = (
            """
            [simulation]
            name = "cli-demo"
            duration = 30.0

            [[spacecraft]]
            name = "sc1"
            """
            + extra
        )
        path = tmp_path / "scenario.toml"
        path.write_text(text.strip() + "\n")
        return path

    def test_validate_prints_subsystem_summary(self, tmp_path, capsys):
        from bskforge.cli import main

        path = self._write(
            tmp_path,
            """
            [[spacecraft.effectors]]
            type = "reactionWheels"
            inertia = 0.001
            [[spacecraft.effectors.wheels]]
            direction_B = [1.0, 0.0, 0.0]

            [spacecraft.fsw.control]
            type = "mrpPD"

            [monte_carlo]
            runs = 2
            """,
        )
        assert main(["validate", str(path)]) == 0
        out = capsys.readouterr().out
        assert "valid: 'cli-demo' — 1 spacecraft, 30 s" in out
        assert "reactionWheels" in out
        assert "fsw mrpPD" in out
        assert "monte carlo: 2 runs" in out

    def test_unknown_key_suggests_field(self, tmp_path, capsys):
        from bskforge.cli import main

        path = (
            self._write(tmp_path)
            .read_text()
            .replace("duration = 30.0", "duratio = 30.0")
        )
        tmp_path.joinpath("scenario.toml").write_text(path)
        assert main(["validate", str(tmp_path / "scenario.toml")]) == 1
        err = capsys.readouterr().err
        assert "unknown key 'duratio'" in err
        assert "did you mean 'duration'" in err

    def test_bad_discriminator_tag_suggests_kind(self, tmp_path, capsys):
        from bskforge.cli import main

        path = self._write(
            tmp_path,
            """
            [[spacecraft.effectors]]
            type = "reactionWheel"
            inertia = 0.001
            """,
        )
        assert main(["validate", str(path)]) == 1
        err = capsys.readouterr().err
        assert "did you mean 'reactionWheels'" in err

    def test_reference_cli_writes_file(self, tmp_path, capsys):
        from bskforge.cli import main

        out = tmp_path / "REF.md"
        assert main(["reference", "--out", str(out)]) == 0
        assert "# `bskforge` scenario format reference" in out.read_text()
        assert main(["reference", "--toml"]) == 0
        assert "bskforge scenario template" in capsys.readouterr().out
