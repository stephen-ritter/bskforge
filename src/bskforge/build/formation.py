#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Formation build phase: barycenters and Hill-frame relative states."""

from __future__ import annotations

from Basilisk.fswAlgorithms import formationBarycenter, hillStateConverter

from bskforge.build.context import SimContext
from bskforge.build.fsw import ensure_trans_msg


def build_formations(ctx: SimContext) -> None:
    """Build every ``[[formations]]`` entry and record its outputs."""
    for form in ctx.config.formations:
        if form.barycenter:
            bary = formationBarycenter.FormationBarycenter()
            bary.ModelTag = f"{form.name}_barycenter"
            bary.mu = ctx.central_mu
            for member in form.members:
                bundle = ctx.bundles[member]
                bary.addSpacecraftToModel(
                    ensure_trans_msg(ctx, bundle), bundle.veh_config_msg
                )
            ctx.add_fsw(bary)
            ctx.register_tag(bary.ModelTag, bary)
            ctx.add_scenario_recorder(
                f"{form.name}.barycenter", bary.transOutMsg, task="fswTask"
            )
        chief = form.relative_to or form.members[0]
        chief_trans = ensure_trans_msg(ctx, ctx.bundles[chief])
        for member in form.members:
            if member == chief:
                continue
            conv = hillStateConverter.hillStateConverter()
            conv.ModelTag = f"{form.name}_{member}_hill"
            conv.chiefStateInMsg.subscribeTo(chief_trans)
            conv.depStateInMsg.subscribeTo(ensure_trans_msg(ctx, ctx.bundles[member]))
            ctx.add_fsw(conv)
            ctx.register_tag(conv.ModelTag, conv)
            ctx.add_scenario_recorder(
                f"{form.name}.rel_{member}", conv.hillStateOutMsg, task="fswTask"
            )
