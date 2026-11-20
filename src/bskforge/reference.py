#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Generated scenario-format reference.

Walks the pydantic JSON schema of :class:`~bskforge.schema.root.
ScenarioConfig` and renders it as a Markdown field reference or a
commented TOML template, so the documentation can never drift from the
schema that actually validates scenarios.
"""

from __future__ import annotations

import json

from bskforge.schema.root import ScenarioConfig

#: schema format version, shown in the generated header
SCHEMA_VERSION = 2


def _schema() -> dict:
    # pydantic caches model_json_schema() per model class
    return ScenarioConfig.model_json_schema()


def _resolve(node: dict) -> dict:
    """Follow ``$ref`` (single-document schema, ``$defs`` root)."""
    seen = set()
    while "$ref" in node:
        name = node["$ref"].rsplit("/", 1)[-1]
        if name in seen:  # pragma: no cover - recursive models
            return {"type": "object"}
        seen.add(name)
        node = _schema()["$defs"][name]
    return node


def _raw_union(node: dict) -> list[dict]:
    """Raw members of an ``anyOf``/``oneOf`` node (unresolved)."""
    node = _resolve(node) if "$ref" in node else node
    return node.get("anyOf") or node.get("oneOf") or []


def _discriminator_kinds(members: list[dict]) -> list[str]:
    kinds: list[str] = []
    for part in members:
        mapping = (part.get("discriminator") or {}).get("mapping", {})
        kinds += sorted(mapping)
    return kinds


def _union_kinds(node: dict) -> list[str]:
    """``type`` discriminator choices of a union node, if any.

    pydantic puts the ``discriminator`` either on the union wrapper
    itself or inside its ``anyOf``/``oneOf`` members depending on the
    model shape; both are accepted here.
    """
    node = _resolve(node) if "$ref" in node else node
    own = sorted((node.get("discriminator") or {}).get("mapping", {}))
    members = node.get("anyOf") or node.get("oneOf") or []
    return own or _discriminator_kinds(members)


def _collapse_union(node: dict) -> dict:
    """Reduce an optional single-model union to that model's schema."""
    members = _raw_union(node)
    real = [m for m in members if m.get("type") != "null"]
    if len(real) == 1 and not _discriminator_kinds(members):
        return _resolve(real[0])
    return _resolve(node)


def _type_name(node: dict) -> str:
    """Short human type label for a property schema node."""
    if "enum" in node:
        return "`" + " / ".join(f"`{v}`" for v in node["enum"]) + "`"
    if node.get("anyOf") or node.get("oneOf"):
        members = _raw_union(node)
        kinds = _union_kinds(node)
        if kinds:
            return "`type` ∈ {" + ", ".join(f"`{k}`" for k in kinds) + "}"
        real = [m for m in members if m.get("type") != "null"]
        if len(real) == 1:
            text = _type_name(real[0])
        else:
            text = " / ".join(_type_name(m) for m in real) or "any"
        if len(real) != len(members):
            text += " / `null`"
        return text
    if "$ref" in node:
        target = _resolve(node)
        if target.get("enum"):
            return "`" + " / ".join(f"`{v}`" for v in target["enum"]) + "`"
        name = node["$ref"].rsplit("/", 1)[-1]
        return f"`{name}`"
    if node.get("type") == "array":
        items = _resolve(node.get("items", {}))
        kinds = _discriminator_kinds(_raw_union(items)) or sorted(
            (items.get("discriminator") or {}).get("mapping", {})
        )
        if kinds:
            return f"array[`type` ∈ {{{', '.join(f'`{k}`' for k in kinds)}}}]"
        return f"array[{_type_name(items)}]"
    if node.get("type") == "object" and not node.get("properties"):
        return "table"
    return {
        "number": "float",
        "integer": "int",
        "string": "str",
        "boolean": "bool",
    }.get(str(node.get("type")), "table")


def _default_text(prop: dict) -> str:
    if "default" not in prop:
        return ""
    default = prop["default"]
    if default is None:
        return "`None`"
    if isinstance(default, str):
        return f"`'{default}'`"
    if isinstance(default, (float, int, bool)):
        return f"`{default}`"
    return f"`{json.dumps(default)}`"


def _summary(description: str | None) -> str:
    """First paragraph of a docstring, plain text."""
    if not description:
        return ""
    line = description.split("\n\n")[0].strip()
    line = " ".join(line.split())
    return line.replace("``", "").replace("`", "")


def _item_schema(prop: dict) -> dict | None:
    """Item schema of an array-of-objects property, else ``None``."""
    node = _resolve(prop)
    if node.get("type") != "array":
        return None
    items = _resolve(node.get("items", {}))
    if items.get("properties") or items.get("anyOf") or items.get("oneOf"):
        return items
    return None


def _field_rows(props: dict, required: set) -> list[str]:
    rows = []
    for key, prop in props.items():
        note = _summary(prop.get("description"))
        rows.append(
            f"| `{key}` | {_type_name(prop)} | {_default_text(prop)} "
            f"| {'✅' if key in required else ''} | {note} |"
        )
    return rows


def _table(node: dict, out: list[str]) -> None:
    doc = _summary(node.get("description"))
    if doc:
        out += [doc]
    out += [
        "",
        "| field | type | default | required | description |",
        "| --- | --- | --- | --- | --- |",
    ]
    out.extend(_field_rows(node["properties"], set(node.get("required", []))))
    out.append("")


def _object_variants(node: dict) -> list[tuple[str, dict]]:
    """``(kind, schema)`` pairs if the node is a *choice* of models.

    A nullable single model (``Model | None``) is not a choice and
    yields ``[]``; discriminated unions yield one pair per variant.
    """
    node = _resolve(node) if "$ref" in node else node
    members = node.get("anyOf") or node.get("oneOf") or []
    if not members or not _union_kinds(node):
        return []
    pairs = []
    for part in members:
        variant = _resolve(part)
        if variant.get("properties"):
            pairs.append(
                (variant["properties"].get("type", {}).get("const", "?"), variant)
            )
    return pairs


def _walk(node: dict, path: str, out: list[str], skip: set[str] | None = None) -> None:
    """Emit the table for ``node`` plus subsections for nested tables.

    ``skip`` names properties whose subsection is rendered elsewhere
    (used to hoist tables shared by all variants of a union).
    """
    skip = skip or set()
    _table(node, out)
    for key, prop in node["properties"].items():
        if key in skip:
            continue
        child = path + "." + key
        variants = _object_variants(prop)
        if variants:
            shared, owners = _shared_variant_tables(variants)
            for kind, variant in variants:
                out.append(f"### `[{child}]` — `{kind}`")
                _walk(variant, child, out, set(shared))
            for shared_key, shared_model in shared.items():
                used = ", ".join(f"`{k}`" for k in owners.get(shared_key, []))
                out.append(f"### `[{child}.{shared_key}]` — used by {used}")
                _walk(shared_model, f"{child}.{shared_key}", out)
            continue
        target = _collapse_union(prop)
        if target.get("type") == "object" and target.get("properties"):
            out.append(f"### `[{child}]`")
            _walk(target, child, out)
            continue
        items = _item_schema(_resolve(prop) if "$ref" in prop else prop)
        if items is not None:
            items = _resolve(items)
            item_variants = _object_variants(items)
            if item_variants:
                shared, owners = _shared_variant_tables(item_variants)
                for kind, variant in item_variants:
                    out.append(f"### `[[{child}]]` — `{kind}`")
                    _walk(variant, child, out, set(shared))
                for shared_key, shared_model in shared.items():
                    used = ", ".join(f"`{k}`" for k in owners.get(shared_key, []))
                    out.append(f"### `[[{child}]].{shared_key}` — used by {used}")
                    _walk(shared_model, f"{child}.{shared_key}", out)
            elif items.get("properties"):
                out.append(f"### `[[{child}]]`")
                _walk(items, child, out)


def _shared_variant_tables(
    variants: list[tuple[str, dict]],
) -> tuple[dict[str, dict], dict[str, list[str]]]:
    """Nested tables reused by two or more variants of one union.

    Returns ``(key -> model, key -> kinds using it)``. Reuse is decided
    by structural equality of the resolved schema: shared models
    resolve to identical subtrees.
    """
    if len(variants) < 2:
        return {}, {}
    seen: dict[str, dict] = {}
    owners: dict[str, list[str]] = {}
    for kind, variant in variants:
        for key, prop in variant["properties"].items():
            target = _collapse_union(prop)
            if not (target.get("type") == "object" and target.get("properties")):
                continue
            if key in seen and seen[key] == target:
                owners[key].append(str(kind))
            elif key not in seen:
                seen[key] = target
                owners[key] = [str(kind)]
    shared = {k: m for k, m in seen.items() if len(owners[k]) > 1}
    return shared, {k: owners[k] for k in shared}


def generate_markdown() -> str:
    """Render the full scenario-format field reference as Markdown."""
    schema = _schema()
    root_props = schema["properties"]
    out = [
        "# `bskforge` scenario format reference",
        "",
        f"Generated from the pydantic schema (v{SCHEMA_VERSION}) — every "
        "field, type and default that `bskforge validate` accepts. "
        "Regenerate with `bskforge reference > REFERENCE.md`.",
        "",
        "Top-level tables: "
        + ", ".join(f"`[{k}]`" for k in root_props)
        + " (`[[spacecraft]]`, `[[comm_links]]` and `[[formations]]` "
        "are repeatable).",
        "",
        "| table | type | description |",
        "| --- | --- | --- |",
    ]
    for key, prop in root_props.items():
        out.append(
            f"| `[{key}]` | {_type_name(prop)} | {_summary(prop.get('description'))} |"
        )
    out.append("")
    for key, prop in root_props.items():
        target = _collapse_union(prop)
        if target.get("type") == "object" and target.get("properties"):
            out.append(f"## `[{key}]`")
            _walk(target, key, out)
            continue
        items = _item_schema(target)
        if items is not None:
            variants = _object_variants(_resolve(items))
            if variants:
                for kind, variant in variants:
                    out.append(f"## `[[{key}]]` — `{kind}`")
                    _walk(variant, key, out)
            elif items.get("properties"):
                out.append(f"## `[[{key}]]`")
                _walk(items, key, out)
    return "\n".join(out).rstrip() + "\n"


def generate_toml_template() -> str:
    """Render a commented TOML skeleton listing every field and default."""
    lines: list[str] = ["# bskforge scenario template (generated)", ""]

    def emit(key: str, prop: dict, path: str, depth: int) -> None:
        node = _resolve(prop)
        pad = "  " * depth
        doc = _summary(prop.get("description"))
        kind = _type_name(prop).replace("`", "")
        default = _default_text(prop).strip("`")
        node = _collapse_union(prop)
        if node.get("properties"):
            lines.append(f"{pad}# -- {path} --")
            if doc:
                lines.append(f"{pad}# {doc}")
            for sub_key, sub in node["properties"].items():
                emit(sub_key, sub, f"{path}.{sub_key}", depth)
        elif _item_schema(node) is not None:
            lines.append(f"{pad}# -- [[{path}]] --")
            if doc:
                lines.append(f"{pad}# {doc}")
            items = _resolve(node.get("items", {}))
            variants = _object_variants(_resolve(items))
            for kind_label, variant in variants or [("", items)]:
                if not variant.get("properties"):
                    continue
                label = f" — {kind_label}" if kind_label else ""
                lines.append(f"{pad}# one entry{label}:")
                for sub_key, sub in variant["properties"].items():
                    emit(sub_key, sub, f"{path}.{sub_key}", depth + 1)
        else:
            comment = f"  # {doc}" if doc else ""
            value = default if default else f"<{kind}>"
            lines.append(f"{pad}# {key} = {value}{comment}")

    for key, prop in _schema()["properties"].items():
        emit(key, prop, key, 0)
    return "\n".join(lines).rstrip() + "\n"
