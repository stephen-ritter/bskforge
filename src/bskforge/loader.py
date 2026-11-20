#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Scenario file loading: TOML parsing, ``extends`` inheritance, validation."""

from __future__ import annotations

import difflib
import tomllib
import typing
from pathlib import Path
from typing import Any

from pydantic import BaseModel, ValidationError

from bskforge.errors import ConfigError
from bskforge.schema.root import ScenarioConfig


def load_scenario(path: str | Path) -> ScenarioConfig:
    """Load, resolve and validate a scenario file.

    Args:
        path: Filesystem path to a scenario TOML file.

    Returns:
        A validated :class:`ScenarioConfig`.

    Raises:
        ConfigError: The file is missing, unparsable, or fails validation.
    """
    path = Path(path)
    if not path.exists():
        raise ConfigError(f"scenario file not found: {path}")
    data = _load_with_extends(path, seen=[])
    return validate_scenario(data, source=str(path))


def validate_scenario(data: dict[str, Any], source: str = "<dict>") -> ScenarioConfig:
    """Validate an in-memory scenario document.

    Args:
        data: Raw scenario mapping (as parsed from TOML).
        source: Label used in error messages.

    Returns:
        A validated :class:`ScenarioConfig`.

    Raises:
        ConfigError: Validation failed; the message carries suggestions.
    """
    data = dict(data)
    data.pop("extends", None)
    try:
        config = ScenarioConfig(**data)
    except ValidationError as exc:
        raise ConfigError(_format_validation_error(exc, data, source)) from exc
    if source != "<dict>":
        config.set_source_path(source)
    return config


def _load_with_extends(path: Path, seen: list[Path]) -> dict[str, Any]:
    """Parse ``path`` and deep-merge any ``extends`` base documents."""
    resolved = path.resolve()
    if resolved in seen:
        chain = " -> ".join(str(p) for p in [*seen, resolved])
        raise ConfigError(f"circular scenario inheritance: {chain}")
    try:
        with open(path, "rb") as handle:
            data = tomllib.load(handle)
    except tomllib.TOMLDecodeError as exc:
        raise ConfigError(f"invalid TOML in {path}: {exc}") from exc

    extends = data.pop("extends", None)
    if extends is None:
        return data
    if isinstance(extends, str):
        extends = [extends]
    if not isinstance(extends, list):
        raise ConfigError(f"{path}: 'extends' must be a string or list of strings")

    merged: dict[str, Any] = {}
    for parent in extends:
        parent_path = (path.parent / str(parent)).resolve()
        if not parent_path.exists():
            raise ConfigError(f"{path}: extended file not found: {parent_path}")
        merged = _deep_merge(merged, _load_with_extends(parent_path, [*seen, resolved]))
    return _deep_merge(merged, data)


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    """Recursively merge ``override`` onto ``base`` (override wins)."""
    out = dict(base)
    for key, value in override.items():
        if key in out and isinstance(out[key], dict) and isinstance(value, dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = value
    return out


# --------------------------------------------------------------------------
# Human-friendly validation errors
# --------------------------------------------------------------------------


def _format_validation_error(
    exc: ValidationError, data: dict[str, Any], source: str
) -> str:
    """Render a pydantic error with did-you-mean hints for unknown keys."""
    lines = [f"invalid scenario {source}:"]
    for err in exc.errors():
        location = ".".join(str(part) for part in err["loc"]) or "<root>"
        message = err["msg"]
        if err["type"] == "extra_forbidden":
            unknown = str(err["loc"][-1])
            known = _known_keys_at(ScenarioConfig, data, err["loc"][:-1])
            close = difflib.get_close_matches(unknown, known, n=1, cutoff=0.5)
            hint = f" (did you mean {close[0]!r} near {location}?)" if close else ""
            message = f"unknown key {unknown!r}{hint}"
        lines.append(f"  {location}: {message}")
    return "\n".join(lines)


def _known_keys_at(
    model: type[BaseModel], data: Any, loc: tuple[Any, ...]
) -> list[str]:
    """Walk the schema along a validated document path to list field names.

    Discriminated unions are resolved by reading the ``type`` value in the
    raw document ``data`` at that position.
    """
    current_model: type[BaseModel] = model
    node: Any = data
    for part in loc:
        if isinstance(part, int):
            if isinstance(node, list) and part < len(node):
                node = node[part]
            continue
        fields = current_model.model_fields
        if part not in fields:
            return []
        annotation = fields[part].annotation
        if isinstance(node, dict) and part in node:
            node = node[part]
        current_model = _descend(annotation, node)
    return list(current_model.model_fields)


def _descend(annotation: Any, node: Any) -> type[BaseModel]:
    """Reduce a field annotation to the concrete model for the given data."""
    origin = typing.get_origin(annotation)
    if origin is list:
        (inner,) = typing.get_args(annotation)
        return _descend(inner, node)
    args = typing.get_args(annotation)
    if args:  # Union / discriminated union
        if isinstance(node, dict) and "type" in node:
            for arg in args:
                model = (
                    arg
                    if isinstance(arg, type) and issubclass(arg, BaseModel)
                    else None
                )
                disc = model.model_fields.get("type") if model else None
                if disc is not None:
                    default = disc.default
                    if default == node["type"]:
                        return arg
        for arg in args:
            if isinstance(arg, type) and issubclass(arg, BaseModel):
                return arg
        return ScenarioConfig
    if isinstance(annotation, type) and issubclass(annotation, BaseModel):
        return annotation
    return ScenarioConfig
