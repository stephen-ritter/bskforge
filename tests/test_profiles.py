#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Load profiles: the escape surface stays available by default and is
refused on request. No Basilisk import required."""

import inspect
from pathlib import Path

import pytest

from bskforge.errors import ConfigError
from bskforge.loader import load_scenario, validate_scenario
from bskforge.profiles import AUDITED, STANDARD, audit_document

EXAMPLES = Path(__file__).resolve().parent.parent / "examples"
HATCH_EXAMPLE = EXAMPLES / "custom_escape_hatch.toml"


def minimal_scenario(**overrides):
    data = {
        "simulation": {"name": "test", "duration": 100.0},
        "spacecraft": [{"name": "sc1"}],
    }
    data.update(overrides)
    return data


def with_sensor_params() -> dict:
    """A clean document except for one device setting a raw attribute."""
    return minimal_scenario(
        spacecraft=[
            {
                "name": "sc1",
                "sensors": [
                    {
                        "type": "magnetometer",
                        "name": "mag1",
                        "params": {"mockRate": 0.5},
                    }
                ],
            }
        ]
    )


class TestStandardProfile:
    """The default must not change: a builder that lost its hatch stopped
    being a builder."""

    def test_the_shipped_escape_hatch_example_still_loads(self):
        config = load_scenario(HATCH_EXAMPLE)
        assert config.custom.tasks or config.custom.modules or config.custom.connections

    def test_params_still_load(self):
        config = validate_scenario(with_sensor_params(), profile=STANDARD)
        assert config.spacecraft[0].sensors[0].params == {"mockRate": 0.5}

    def test_profile_is_keyword_only_so_it_cannot_be_mistaken_for_a_path(self):
        """``load_scenario(toml, "audited")`` must not be a thing anyone can type.

        A positional second argument would be a footgun whose failure mode is
        silence: the misspelled call would still load, permissively. The
        signature is the guarantee, so the signature is what gets checked.
        """
        for function in (load_scenario, validate_scenario):
            parameter = inspect.signature(function).parameters["profile"]
            assert parameter.kind is inspect.Parameter.KEYWORD_ONLY


class TestAuditedProfile:
    def test_a_declared_scenario_loads(self):
        config = validate_scenario(minimal_scenario(), profile=AUDITED)
        assert config.spacecraft[0].name == "sc1"

    def test_the_escape_section_is_refused_by_name(self):
        with pytest.raises(ConfigError) as excinfo:
            load_scenario(HATCH_EXAMPLE, profile=AUDITED)
        message = str(excinfo.value)
        assert "custom" in message
        assert "declares Basilisk primitives directly" in message

    def test_a_device_attribute_table_is_refused_with_its_path(self):
        with pytest.raises(ConfigError) as excinfo:
            validate_scenario(with_sensor_params(), source="sc.toml", profile=AUDITED)
        message = str(excinfo.value)
        assert "sc.toml" in message
        assert "spacecraft[0].sensors[0].params" in message

    def test_every_use_is_listed_at_once(self):
        """Two holes are one decision, so the message names both."""
        data = {
            "simulation": {"name": "test", "duration": 10.0},
            "spacecraft": [
                {
                    "name": "sc1",
                    "effectors": [{"type": "mtb", "name": "mtb1", "params": {}}],
                }
            ],
            "custom": {"tasks": [{"name": "t", "rate": 1.0}]},
        }
        with pytest.raises(ConfigError) as excinfo:
            validate_scenario(data, profile=AUDITED)
        message = str(excinfo.value)
        assert "2 use(s)" in message
        assert "custom" in message
        assert "spacecraft[0].effectors[0].params" in message

    def test_an_empty_table_is_still_refused(self):
        """Presence is the rule, not content.

        ``params = {}`` sets nothing today, which is exactly why a content test
        would rot: the audit would pass, someone would add a key later, and the
        profile would stop meaning "the schema names everything".
        """
        device = {"type": "magnetometer", "name": "m", "params": {}}
        data = minimal_scenario(spacecraft=[{"name": "sc1", "sensors": [device]}])
        with pytest.raises(ConfigError, match="params"):
            validate_scenario(data, profile=AUDITED)

    def test_inheritance_cannot_launder_a_hatch(self, tmp_path):
        """The audit sees the merged document, not the file that was named."""
        base = tmp_path / "base.toml"
        base.write_text(
            '[simulation]\nname = "b"\nduration = 10.0\n\n'
            '[[spacecraft]]\nname = "sc1"\n\n'
            '[[spacecraft.sensors]]\ntype = "magnetometer"\nname = "m"\n'
            "[[spacecraft.sensors.params]]\nmockRate = 0.5\n",
            encoding="utf-8",
        )
        child = tmp_path / "child.toml"
        child.write_text(
            'extends = "base.toml"\n\n[simulation]\nname = "child"\n',
            encoding="utf-8",
        )

        with pytest.raises(ConfigError, match="params"):
            load_scenario(child, profile=AUDITED)

    def test_the_hatch_is_refused_at_the_root_only(self):
        """A nested key of the same name is not the escape section.

        Over-refusal would be its own bug: it would refuse documents that never
        touched the feature, and a guard that cries wolf gets switched off.
        """
        nested = {"spacecraft": [{"name": "sc1", "custom": {"x": 1}}]}
        audit_document(nested, profile=AUDITED)


class TestProfileName:
    def test_a_typo_is_refused_rather_than_loading_as_standard(self):
        """The one failure mode a profile system has to make impossible."""
        with pytest.raises(ConfigError, match="unknown scenario profile"):
            validate_scenario(minimal_scenario(), profile="auditd")

    def test_the_message_lists_the_known_profiles(self):
        with pytest.raises(ConfigError) as excinfo:
            validate_scenario(minimal_scenario(), profile="strict")
        assert "standard" in str(excinfo.value)
        assert "audited" in str(excinfo.value)
