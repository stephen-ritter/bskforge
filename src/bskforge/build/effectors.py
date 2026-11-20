#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Effector build phase: reaction wheels via ``simIncludeRW``."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from Basilisk.simulation import reactionWheelStateEffector
from Basilisk.utilities import simIncludeRW

from bskforge.build.context import RwBundle, SimContext, SpacecraftBundle
from bskforge.errors import BuildError
from bskforge.schema.effectors import ReactionWheelsConfig, WheelConfig

#: Jitter model label -> Basilisk RWModel enum.
JITTER_MODELS = {
    "none": reactionWheelStateEffector.BalancedWheels,
    "constant": reactionWheelStateEffector.JitterSimple,
    "speedDependent": reactionWheelStateEffector.JitterFullyCoupled,
}

EFFECTOR_BUILDERS: dict[str, Callable[..., Any]] = {}


def effector_builder(type_name: str) -> Callable[[Callable], Callable]:
    """Register a builder function for an effector ``type``."""

    def _decorator(fn: Callable) -> Callable:
        EFFECTOR_BUILDERS[type_name] = fn
        return fn

    return _decorator


@effector_builder("reactionWheels")
def build_reaction_wheels(
    ctx: SimContext, bundle: SpacecraftBundle, cfg: ReactionWheelsConfig
) -> None:
    """Attach a reaction-wheel state effector built from ``simIncludeRW``.

    Args:
        ctx: Simulation build context.
        bundle: The vehicle this RW set belongs to.
        cfg: Reaction-wheel set configuration.
    """
    if not cfg.wheels:
        raise BuildError(
            f"spacecraft {bundle.config.name!r}: reactionWheels effector has no wheels"
        )

    factory = simIncludeRW.rwFactory()
    for index, wheel in enumerate(cfg.wheels):
        label = _wheel_label(wheel, index)
        factory.create(
            "custom",
            list(wheel.direction_B),
            Js=cfg.inertia,
            maxMomentum=cfg.inertia * wheel.max_speed,
            Omega=wheel.initial_speed,  # [rpm]
            u_max=wheel.max_torque if wheel.max_torque is not None else 1e10,
            RWModel=JITTER_MODELS[wheel.jitter_model],
            label=label,
            **wheel.params,
        )

    rw_effector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rw_effector.ModelTag = f"{bundle.config.name}_rwEff"
    factory.addToSpacecraft(rw_effector.ModelTag, rw_effector, bundle.sc)
    ctx.add_dyn(rw_effector)
    ctx.register_tag(rw_effector.ModelTag, rw_effector)

    bundle.rw = RwBundle(
        effector=rw_effector,
        factory=factory,
        cfg_msg=factory.getConfigMessage(),
        recorder_cfg=cfg.recorder,
    )


def _wheel_label(wheel: WheelConfig, index: int) -> str:
    """Return a Basilisk-valid (<=5 char) RW device label."""
    return (wheel.name or f"RW{index}")[:5]
