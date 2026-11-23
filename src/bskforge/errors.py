#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Errors raised by bskforge."""

from __future__ import annotations


class BskForgeError(Exception):
    """Base class for all bskforge errors."""


class ConfigError(BskForgeError):
    """A scenario file failed schema validation or resolution."""


class AuditError(ConfigError):
    """A document used the escape surface under a profile that forbids it.

    Its own type because the consumer that asked for ``profile="audited"`` is
    the one party that can say what the refusal costs *in its own system*, and
    it needs to catch this refusal without also swallowing typos in field names.
    """


class UnsupportedError(BskForgeError):
    """A configured feature is not yet covered by the current phase."""


class BuildError(BskForgeError):
    """The Basilisk simulation graph could not be constructed."""
