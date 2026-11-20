#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Vehicle build phase: hub, orbit/attitude initial state, force/torque port."""

from __future__ import annotations

from typing import Any

import numpy as np
from Basilisk.architecture import messaging
from Basilisk.simulation import extForceTorque, spacecraft
from Basilisk.utilities import macros, orbitalMotion, simHelpers

from bskforge.build.context import SimContext, SpacecraftBundle
from bskforge.build.registry import apply_params
from bskforge.schema.spacecraft import SpacecraftSection


def build_vehicle(ctx: SimContext, sc_cfg: SpacecraftSection) -> SpacecraftBundle:
    """Create one spacecraft rigid body and its initial-state configuration.

    The vehicle is added to the dynamics task, connected to the shared
    gravity bodies, and given an ``extForceTorque`` effector so later FSW
    phases can inject commanded torque.

    Args:
        ctx: The simulation build context.
        sc_cfg: Validated spacecraft configuration.

    Returns:
        The assembled per-vehicle bundle (registered in ``ctx``).
    """
    sc = spacecraft.Spacecraft()
    sc.ModelTag = sc_cfg.name

    hub = sc_cfg.hub
    sc.hub.mHub = hub.mass
    flat_inertia = list(hub.inertia)
    sc.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(
        np.array(flat_inertia, dtype=float)
    )
    sc.hub.r_BcB_B = [[v] for v in hub.r_BcB_B]
    apply_params(sc, hub.params, f"spacecraft {sc_cfg.name!r} hub")

    _set_orbit_state(sc, sc_cfg, ctx.central_mu)
    _set_attitude_state(sc, sc_cfg)

    ctx.grav_factory.addBodiesTo(sc)
    ctx.add_dyn(sc)
    ctx.register_tag(sc_cfg.name, sc)

    ext_ft = extForceTorque.ExtForceTorque()
    ext_ft.ModelTag = f"{sc_cfg.name}_extFT"
    sc.addDynamicEffector(ext_ft)
    ctx.add_dyn(ext_ft)
    ctx.register_tag(ext_ft.ModelTag, ext_ft)

    veh_config_msg = messaging.VehicleConfigMsg()
    veh_config_msg.write(
        messaging.VehicleConfigMsgPayload(  # ty: ignore
            ISCPntB_B=flat_inertia, massSC=sc_cfg.hub.mass
        )
    )

    bundle = SpacecraftBundle(
        config=sc_cfg, sc=sc, ext_ft=ext_ft, veh_config_msg=veh_config_msg
    )
    ctx.bundles[sc_cfg.name] = bundle
    return bundle


def _set_orbit_state(sc: Any, sc_cfg: SpacecraftSection, central_mu: float) -> None:
    """Apply translational initial state from ``[spacecraft.orbit]``."""
    if sc_cfg.orbit is None:
        return
    orbit = sc_cfg.orbit
    if orbit.type == "elements":
        elems = orbit.elements
        oe = orbitalMotion.ClassicElements()
        oe.a = elems.a
        oe.e = elems.e
        oe.i = elems.i * macros.D2R
        oe.Omega = elems.raan * macros.D2R
        oe.omega = elems.omega * macros.D2R
        oe.f = elems.trueAnomaly * macros.D2R
        r_N, v_N = orbitalMotion.elem2rv(central_mu, oe)
        sc.hub.r_CN_NInit = [[float(c)] for c in r_N]
        sc.hub.v_CN_NInit = [[float(c)] for c in v_N]
    elif orbit.type == "rv":
        sc.hub.r_CN_NInit = [[v] for v in orbit.rv.position]
        sc.hub.v_CN_NInit = [[v] for v in orbit.rv.velocity]


def _set_attitude_state(sc: Any, sc_cfg: SpacecraftSection) -> None:
    """Apply rotational initial state from ``[spacecraft.attitude]``."""
    if sc_cfg.attitude is None or sc_cfg.attitude.initial is None:
        return
    initial = sc_cfg.attitude.initial
    sc.hub.sigma_BNInit = [[v] for v in initial.sigma_BN]
    sc.hub.omega_BN_BInit = [[v] for v in initial.omega_BN_B]
