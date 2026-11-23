#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Load profiles: how much a scenario document may express for itself.

bskforge is a builder, and a builder needs an escape hatch. ``[custom]`` exposes
the Basilisk primitives that the domain sections compile down to, and a device's
``params`` table sets an attribute the schema does not name yet. Both are how a
mission uses a capability of the installed Basilisk build before the schema grows
a block for it, and taking them away would stop that - including the route by
which a prototype becomes a declared type.

A *consumer* that grades a scenario, however, can only grade what the schema
names. Hardware wired in by hand has no type, so nothing downstream - flight
software, ground identity, an acceptance gate - can say what it is or check what
it produces. Its readings exist and nothing judges them, which fails quietly
rather than loudly. ``profile="audited"`` is the request to refuse such a
document at load, where the answer is a sentence, instead of three layers later
where it is an investigation.
"""

from __future__ import annotations

from typing import Any

from bskforge.errors import AuditError

#: The ordinary profile: the escape surface is available, as a builder should allow.
STANDARD = "standard"

#: Every device must be a declared type; the escape surface is refused.
AUDITED = "audited"

#: The profiles :func:`bskforge.loader.load_scenario` accepts.
PROFILES = (STANDARD, AUDITED)

# Root-level sections that bypass the domain schema entirely. They are refused
# only at the root, because a nested key of the same name is not this feature.
_ESCAPE_SECTIONS = frozenset({"custom"})

# The per-device attribute table that bypasses the device schema, at any depth.
_PARAM_TABLE = "params"

_WHY = {
    "custom": "declares Basilisk primitives directly instead of a modelled device",
    "params": "sets Basilisk attributes that the device's schema does not name",
}


def audit_document(
    data: dict[str, Any], source: str = "<dict>", profile: str = AUDITED
) -> None:
    """Refuse a document that leaves the declared schema.

    Args:
        data: the scenario document, with ``extends`` already resolved.
        source: label used in the message.
        profile: the profile being enforced, named in the message.

    Raises:
        AuditError: every use of the escape surface, listed together. A gate
            that reports one violation per trip teaches slowly, and a document
            with two of them is one decision, not two.
    """
    uses = _escape_uses(data)
    if not uses:
        return

    lines = [
        f"scenario {source} cannot be loaded with profile '{profile}': "
        f"{len(uses)} use(s) of the escape surface",
    ]
    lines += [f"  {where}  - {_WHY[what]}" for what, where in uses]
    lines.append(
        f"profile '{profile}' requires every device to be a declared type, so "
        "that whatever consumes this scenario can name it and judge what it "
        "produces. If the type you need does not exist yet: prototype it with "
        "the escape surface in a development scenario, add the type upstream "
        "with tests, then declare it here."
    )
    raise AuditError("\n".join(lines))


def _escape_uses(data: Any, path: str = "") -> list[tuple[str, str]]:
    """Every ``(what, where)`` pair where the document steps outside the schema.

    An escape section is reported once, at the root, and not descended into:
    ``params`` inside a hand-written primitive is the same decision as the
    primitive itself, and listing both would read like two problems.
    """
    uses: list[tuple[str, str]] = []
    if isinstance(data, dict):
        for key, value in data.items():
            here = f"{path}.{key}" if path else str(key)
            if (not path and key in _ESCAPE_SECTIONS) or key == _PARAM_TABLE:
                uses.append((str(key), here))
                continue
            uses += _escape_uses(value, here)
    elif isinstance(data, list):
        for index, item in enumerate(data):
            uses += _escape_uses(item, f"{path}[{index}]")
    return uses
