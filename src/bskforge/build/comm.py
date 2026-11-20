#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Comm build phase: RF antennas and link budgets between vehicles."""

from __future__ import annotations

from math import radians
from typing import Any

from Basilisk.architecture import messaging
from Basilisk.simulation import groundLocation, linkBudget, simpleAntenna

from bskforge.build.context import SimContext
from bskforge.schema.comm import AntennaConfig

_ANTENNA_STATE = {
    "tx": simpleAntenna.ANTENNA_TX,
    "rx": simpleAntenna.ANTENNA_RX,
}


def build_comm_links(ctx: SimContext) -> None:
    """Build every ``[[comm_links]]`` entry and record its link budget."""
    for link in ctx.config.comm_links:
        tx = _make_antenna(ctx, f"{link.name}_tx", link.tx_antenna, "tx")
        _mount_on_vehicle(ctx, tx, link.transmitter)
        if link.type == "downlink":
            station = groundLocation.GroundLocation()
            station.ModelTag = f"{link.name}_station"
            station.planetRadius = float(ctx.central_body.radEquator)
            station.specifyLocation(
                radians(link.ground_station.lat),
                radians(link.ground_station.long),
                link.ground_station.elevation,
            )
            station.minimumElevation = radians(link.ground_station.min_elevation)
            station.planetInMsg.subscribeTo(ctx.central_state_msg())
            station.addSpacecraftToModel(ctx.bundles[link.transmitter].sc.scStateOutMsg)
            ctx.add_dyn(station)
            ctx.register_tag(station.ModelTag, station)
            rx = _make_antenna(ctx, f"{link.name}_rx", link.rx_antenna, "rx")
            rx.groundStateInMsg.subscribeTo(station.currentGroundStateOutMsg)
        else:
            rx = _make_antenna(ctx, f"{link.name}_rx", link.rx_antenna, "rx")
            _mount_on_vehicle(ctx, rx, link.receiver)
        budget = linkBudget.LinkBudget()
        budget.ModelTag = f"{link.name}_linkBudget"
        budget.antennaInPayload_1.subscribeTo(tx.antennaOutMsg)
        budget.antennaInPayload_2.subscribeTo(rx.antennaOutMsg)
        ctx.add_dyn(budget)
        ctx.register_tag(budget.ModelTag, budget)
        ctx.add_scenario_recorder(f"{link.name}.link", budget.linkBudgetOutPayload)
        ctx.add_scenario_recorder(f"{link.name}.rx", rx.antennaOutMsg)
        if link.type == "downlink":
            station = ctx.modules_by_tag[f"{link.name}_station"]
            ctx.add_scenario_recorder(f"{link.name}.access", station.accessOutMsgs[0])


def _make_antenna(
    ctx: SimContext,
    tag: str,
    cfg: AntennaConfig,
    state: str,
) -> Any:
    """Create one ``SimpleAntenna`` with its static state message."""
    ant = simpleAntenna.SimpleAntenna()
    ant.ModelTag = tag
    ant.setAntennaName(tag)
    ant.setAntennaFrequency(cfg.frequency)
    ant.setAntennaBandwidth(cfg.bandwidth)
    ant.setAntennaDirectivity_dB(cfg.directivity_db)
    ant.setAntennaHpbwRatio(cfg.hpbw_ratio)
    ant.setAntennaP_Tx(cfg.tx_power)
    ant.setAntennaRadEfficiency(cfg.radial_efficiency)
    ant.setAntennaEquivalentNoiseTemp(cfg.noise_temperature)
    ant.setAntennaP_Rx(0.0)
    ant.setAntennaPositionBodyFrame(list(cfg.position_B))
    ant.setAntennaOrientationBodyFrame(list(cfg.orientation_B))
    if cfg.ambient_temperature is not None:
        ant.setAntennaEnvironmentTemperature(cfg.ambient_temperature)
    state_payload = messaging.AntennaStateMsgPayload()
    state_payload.antennaState = _ANTENNA_STATE[state]
    state_msg = messaging.AntennaStateMsg().write(state_payload)
    ant.antennaSetStateInMsg.subscribeTo(state_msg)
    ant.sunInMsg.subscribeTo(ctx.ensure_sun_msg())
    ctx.add_dyn(ant)
    ctx.register_tag(ant.ModelTag, ant)
    return ant


def _mount_on_vehicle(ctx: SimContext, antenna: Any, sc_name: str) -> None:
    """Feed the antenna from the spacecraft hub state message."""
    antenna.scStateInMsg.subscribeTo(ctx.bundles[sc_name].sc.scStateOutMsg)
