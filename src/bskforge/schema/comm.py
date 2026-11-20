#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  See LICENSE for full terms.

"""Communication links: RF antennas plus the Basilisk link-budget module."""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import Field

from bskforge.schema.base import ScenarioModel, Vec3


class AntennaConfig(ScenarioModel):
    """One RF antenna end of a link (``simpleAntenna.SimpleAntenna``).

    Attributes:
        frequency: Center frequency [Hz].
        bandwidth: Receiver bandwidth [Hz].
        directivity_db: Antenna directivity ``D`` [dB].
        hpbw_ratio: Half-power-beamwidth azimuth/elevation ratio ``k``.
        tx_power: Transmit feed power [W].
        radial_efficiency: Radiation efficiency eta_r [-].
        noise_temperature: Equivalent system noise temperature [K].
        position_B: Antenna phase-centre position in the body frame [m].
        orientation_B: MRP of the antenna frame A relative to the
            equipment body frame the link is mounted in.
        ambient_temperature: Environment temperature [K]; the module's
            own thermal model is used when unset.
    """

    frequency: float = Field(8.4e9, gt=0, description="Center frequency [Hz]")
    bandwidth: float = Field(20e6, gt=0, description="Bandwidth [Hz]")
    directivity_db: float = Field(20.0, description="Directivity [dB]")
    hpbw_ratio: float = Field(1.0, gt=0, description="HPBW ratio k")
    tx_power: float = Field(100.0, ge=0, description="Transmit power [W]")
    radial_efficiency: float = Field(0.95, gt=0, le=1)
    noise_temperature: float = Field(290.0, gt=0, description="System noise temp [K]")
    position_B: Vec3 = [0.0, 0.0, 1.0]
    orientation_B: Vec3 = [0.0, 0.0, 0.0]
    ambient_temperature: float | None = Field(None)


class GroundStationConfig(ScenarioModel):
    """A site on the central body (``groundLocation.GroundLocation``).

    Attributes:
        lat: Geodetic latitude [deg].
        long: East longitude [deg].
        elevation: Height above the reference sphere [m].
        min_elevation: Minimum elevation angle for access [deg].
    """

    lat: float = Field(ge=-90, le=90, description="Latitude [deg]")
    long: float = Field(description="East longitude [deg]")
    elevation: float = Field(0.0, description="Elevation [m]")
    min_elevation: float = Field(0.0, description="Min access elevation [deg]")


class DownlinkConfig(ScenarioModel):
    """Spacecraft-to-ground RF link: ``type = "downlink"``.

    Requires ``[world.spice]``: the ground station rides a rotating,
    ephemeris-driven planet.
    """

    type: Literal["downlink"] = "downlink"
    name: str
    transmitter: str = Field(description="Transmitting spacecraft name")
    ground_station: GroundStationConfig
    tx_antenna: AntennaConfig = Field(default_factory=AntennaConfig)
    rx_antenna: AntennaConfig = Field(default_factory=AntennaConfig)


class CrosslinkConfig(ScenarioModel):
    """Spacecraft-to-spacecraft RF link: ``type = "crosslink"``."""

    type: Literal["crosslink"] = "crosslink"
    name: str
    transmitter: str = Field(description="Transmitting spacecraft name")
    receiver: str = Field(description="Receiving spacecraft name")
    tx_antenna: AntennaConfig = Field(default_factory=AntennaConfig)
    rx_antenna: AntennaConfig = Field(default_factory=AntennaConfig)


CommLinkConfig = Annotated[
    DownlinkConfig | CrosslinkConfig,
    Field(discriminator="type"),
]
