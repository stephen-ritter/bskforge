#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Environment guard: reject Phase-2+ models until they are implemented."""

from __future__ import annotations

from bskforge.build.context import SimContext
from bskforge.errors import UnsupportedError


def check_environment_support(ctx: SimContext) -> None:
    """Fail with a clear message if a scenario requests unbuilt environment.

    Args:
        ctx: Simulation build context.

    Raises:
        UnsupportedError: Environment models or disturbances are requested.
    """
    config = ctx.config
    requested: list[str] = []
    if config.environment is not None and any(
        getattr(config.environment, field) is not None
        for field in ("magnetic_field", "atmosphere", "eclipse", "solar_flux")
    ):
        requested.append("[environment]")
    for bundle_cfg in config.spacecraft:
        if bundle_cfg.disturbances:
            requested.append(f"[[spacecraft]] {bundle_cfg.name}.disturbances")
        if bundle_cfg.environment is not None:
            requested.append(
                f"[[spacecraft]] {bundle_cfg.name}.environment"
            )
    if requested:
        raise UnsupportedError(
            "environment models land in Phase 2; requested today: "
            + ", ".join(sorted(set(requested)))
        )
