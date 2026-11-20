#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of
#  Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for
#  any purpose with or without fee is hereby granted, provided that the
#  above copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

"""Simulation runner that executes Basilisk simulations from TOML config."""

from __future__ import annotations

from pathlib import Path
from typing import Union

from Basilisk import __path__
from Basilisk.architecture import astroConstants, messaging
from Basilisk.fswAlgorithms import (
    attTrackingError,
    inertial3D,
    mrpFeedback,
    rwMotorTorque,
)
from Basilisk.simulation import (
    extForceTorque,
    reactionWheelStateEffector,
    simpleNav,
    spacecraft,
    vizSupport,
)
from Basilisk.utilities import (
    SimulationBaseClass,
    macros,
    orbitalMotion,
    simHelpers,
    simIncludeGravBody,
    simIncludeRW,
)

from bsk_toml_sim.models.simulation import SimulationConfig

BSK_PATH = __path__[0]


class SimulationResult:
    """Container for simulation output data.

    Attributes:
        sim: The executed SimBaseClass instance.
        satellites: List of per-satellite result dicts with logged data.
    """

    def __init__(self, sim, satellite_results: list[dict]) -> None:
        self.sim = sim
        self.satellites = satellite_results


class SimulationRunner:
    """Builds and runs a Basilisk simulation from a SimulationConfig.

    The runner constructs the full Basilisk message graph for each satellite
    defined in the config: spacecraft dynamics, gravity bodies, simpleNav,
    inertial3D guidance, attitude tracking error, MRP feedback control, and
    optionally reaction wheel effectors with rwMotorTorque allocation.

    Args:
        config: Validated simulation configuration.

    Example::

        from bsk_toml_sim import load_simulation_config, SimulationRunner

        config = load_simulation_config("my_sim.toml")
        runner = SimulationRunner(config)
        result = runner.run()
    """

    def __init__(self, config: SimulationConfig) -> None:
        self.config = config
        # Populated during _build_simulation for visualization
        self._sc_objects = []
        self._sim_task_name = None
        self._rw_state_effectors = []

    def run(self) -> SimulationResult:
        """Execute the simulation.

        Visualization is enabled when ``simulation.visualization.enabled``
        is set to true in the TOML config.

        Returns:
            SimulationResult with logged data for each satellite.
        """
        sim = self._build_simulation()
        self._configure_stop_time(sim)

        if self._is_visualization_enabled():
            self._enable_visualization(sim)

        sim.InitializeSimulation()
        sim.ExecuteSimulation()

        return self._collect_results(sim)

    def _is_visualization_enabled(self):
        """Return True when the TOML config requests visualization."""
        if self.config.visualization is not None:
            return self.config.visualization.enabled
        return False

    # ------------------------------------------------------------------
    # Internal build steps
    # ------------------------------------------------------------------

    def _build_simulation(self):
        """Construct the full Basilisk simulation graph."""
        sim_task_name = "simTask"
        sim_process_name = "simProcess"

        self._sim_task_name = sim_task_name
        self._sc_objects = []
        self._rw_state_effectors = []

        sc_sim = SimulationBaseClass.SimBaseClass()
        sim_time_step = macros.sec2nano(self.config.dt_dynamics)

        dyn_process = sc_sim.CreateNewProcess(sim_process_name)
        dyn_process.addTask(sc_sim.CreateNewTask(sim_task_name, sim_time_step))

        # Build gravity bodies once (shared central body)
        grav_factory = self._setup_gravity(sc_sim, sim_task_name)

        # Build each satellite
        for sat_cfg in self.config.satellites:
            self._build_satellite(
                sc_sim, sim_task_name, sat_cfg, grav_factory
            )

        return sc_sim

    def _setup_gravity(self, sc_sim, task_name):
        """Create gravity body factory with central body."""
        grav_factory = simIncludeGravBody.gravBodyFactory()
        central_name = self.config.get_central_body_name()

        if central_name == "earth":
            earth = grav_factory.createEarth()
            earth.isCentralBody = True
        elif central_name == "mars":
            mars = grav_factory.createMars()
            mars.isCentralBody = True
        elif central_name == "sun":
            sun = grav_factory.createSun()
            sun.isCentralBody = True
        else:
            earth = grav_factory.createEarth()
            earth.isCentralBody = True

        # Add additional gravity bodies if specified
        for body_cfg in self.config.gravity_bodies or []:
            name = body_cfg.name
            is_central = body_cfg.is_central_body
            if name == "sun" and not is_central:
                grav_factory.createSun()
            elif name == "moon" and not is_central:
                grav_factory.createMoon()
            elif name == "mars" and not is_central:
                grav_factory.createMars()

        return grav_factory

    def _build_satellite(self, sc_sim, task_name, sat_cfg, grav_factory):
        """Build one satellite's simulation subgraph."""
        model_tag = sat_cfg.name

        # --- Spacecraft dynamics module ---
        sc_obj = self._create_spacecraft(sat_cfg)
        self._sc_objects.append(sc_obj)
        grav_factory.addBodiesTo(sc_obj)
        sc_sim.AddModelToTask(task_name, sc_obj)

        # --- External force/torque (control + disturbances) ---
        ext_ft = extForceTorque.ExtForceTorque()
        ext_ft.ModelTag = f"{model_tag}_extFT"
        sc_obj.addDynamicEffector(ext_ft)
        sc_sim.AddModelToTask(task_name, ext_ft)

        # --- Reaction wheel state effector (if configured) ---
        rw_state_eff = None
        rw_speed_msg = None
        if sat_cfg.effectors.reaction_wheels is not None:
            rw_state_eff, rw_speed_msg = self._add_reaction_wheels(
                sc_sim, task_name, sc_obj, sat_cfg
            )
            if rw_state_eff is not None:
                self._rw_state_effectors.append(rw_state_eff)

        # --- Simple navigation (truth) ---
        s_nav = simpleNav.SimpleNav()
        s_nav.ModelTag = f"{model_tag}_simpleNav"
        sc_sim.AddModelToTask(task_name, s_nav)
        s_nav.scStateInMsg.subscribeTo(sc_obj.scStateOutMsg)

        # --- Inertial guidance reference ---
        inertial_guidance = inertial3D.inertial3D()
        inertial_guidance.ModelTag = f"{model_tag}_inertial3D"
        sc_sim.AddModelToTask(task_name, inertial_guidance)
        inertial_guidance.sigma_R0N = [0.0, 0.0, 0.0]

        # --- Attitude tracking error ---
        att_error = attTrackingError.attTrackingError()
        att_error.ModelTag = f"{model_tag}_attError"
        sc_sim.AddModelToTask(task_name, att_error)
        att_error.attNavInMsg.subscribeTo(s_nav.attOutMsg)
        att_error.attRefInMsg.subscribeTo(inertial_guidance.attRefOutMsg)

        # --- MRP feedback control ---
        mrp_ctrl = self._setup_mrp_feedback(sat_cfg, model_tag)
        sc_sim.AddModelToTask(task_name, mrp_ctrl)
        mrp_ctrl.guidInMsg.subscribeTo(att_error.attGuidOutMsg)

        # --- Vehicle config message ---
        I_flat = sat_cfg.hub.inertia.to_flat()
        veh_config_payload = messaging.VehicleConfigMsgPayload(ISCPntB_B=list(I_flat))
        veh_config_msg = messaging.VehicleConfigMsg()
        veh_config_msg.write(veh_config_payload)
        mrp_ctrl.vehConfigInMsg.subscribeTo(veh_config_msg)

        # --- RW config message (if RWs present) ---
        rw_config_msg = None
        if rw_state_eff is not None:
            rw_config_msg = self._create_rw_config_message(sat_cfg, model_tag)
            mrp_ctrl.rwParamsInMsg.subscribeTo(rw_config_msg)
            if rw_speed_msg is not None:
                mrp_ctrl.rwSpeedsInMsg.subscribeTo(rw_speed_msg)

            # --- RW motor torque allocation ---
            rw_motor = rwMotorTorque.rwMotorTorque()
            rw_motor.ModelTag = f"{model_tag}_rwMotorTorque"
            sc_sim.AddModelToTask(task_name, rw_motor)
            rw_motor.vehControlInMsg.subscribeTo(mrp_ctrl.cmdTorqueOutMsg)
            rw_motor.rwParamsInMsg.subscribeTo(rw_config_msg)
            rw_state_eff.rwMotorCmdInMsg.subscribeTo(
                rw_motor.rwMotorTorqueOutMsg
            )

        # --- Connect control torque to extForceTorque ---
        ext_ft.cmdTorqueInMsg.subscribeTo(mrp_ctrl.cmdTorqueOutMsg)

        # --- Data logging (per-config recorders) ---
        self._setup_logging(sc_sim, task_name, sc_obj, mrp_ctrl, rw_state_eff, sat_cfg)

    def _create_spacecraft(self, sat_cfg):
        """Create and configure the spacecraft dynamics module."""
        sc_obj = spacecraft.Spacecraft()
        sc_obj.ModelTag = sat_cfg.name

        # Hub properties
        sc_obj.hub.mHub = sat_cfg.hub.mass  # [kg]
        sc_obj.hub.r_BcB_B = [
            [sat_cfg.hub.r_BcB_B.x],
            [sat_cfg.hub.r_BcB_B.y],
            [sat_cfg.hub.r_BcB_B.z],
        ]
        sc_obj.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(
            sat_cfg.hub.inertia.to_flat()
        )

        # Initial orbit state
        self._set_orbit_state(sc_obj, sat_cfg)

        # Initial attitude state
        if sat_cfg.attitude and sat_cfg.attitude.initial:
            init = sat_cfg.attitude.initial
            sc_obj.hub.sigma_BNInit = [[s] for s in init.sigma_BN]
            sc_obj.hub.omega_BN_BInit = [[w] for w in init.omega_BN_B]

        return sc_obj

    def _set_orbit_state(self, sc_obj, sat_cfg):
        """Set initial orbital state from config."""
        central_name = self.config.get_central_body_name()
        mu = self._get_gravitational_parameter(central_name)

        orbit = sat_cfg.orbit
        if orbit.type == "elements":
            elems = orbit.get_elements()
            oe = orbitalMotion.ClassicElements()
            oe.a = elems.semi_major_axis
            oe.e = elems.eccentricity
            oe.i = elems.inclination * macros.D2R
            oe.Omega = elems.raan * macros.D2R
            oe.omega = elems.arg_perigee * macros.D2R
            oe.f = elems.true_anomaly * macros.D2R
            rN, vN = orbitalMotion.elem2rv(mu, oe)
            sc_obj.hub.r_CN_NInit = rN
            sc_obj.hub.v_CN_NInit = vN
        elif orbit.type == "rv":
            rv = orbit.get_rv()
            sc_obj.hub.r_CN_NInit = [
                [rv.position.x],
                [rv.position.y],
                [rv.position.z],
            ]
            sc_obj.hub.v_CN_NInit = [
                [rv.velocity.x],
                [rv.velocity.y],
                [rv.velocity.z],
            ]

    def _get_gravitational_parameter(self, body_name: str) -> float:
        """Return gravitational parameter mu for the named body."""
        mu_map = {
            "earth": astroConstants.MU_EARTH * 1e9,  # [m^3/s^2]
            "mars": 4.282837e13,  # [m^3/s^2]
            "sun": astroConstants.MU_SUN * 1e9,  # [m^3/s^2]
        }
        return mu_map.get(body_name, astroConstants.MU_EARTH * 1e9)

    def _inertia_to_eigen(self, inertia):
        """Convert InertiaMatrix to Eigen-style column-major 3x3."""
        flat = inertia.to_flat()
        return [
            [flat[0], flat[3], flat[6]],
            [flat[1], flat[4], flat[7]],
            [flat[2], flat[5], flat[8]],
        ]

    def _setup_mrp_feedback(self, sat_cfg, model_tag):
        """Create and configure the MRP feedback control module."""
        mrp_ctrl = mrpFeedback.mrpFeedback()
        mrp_ctrl.ModelTag = f"{model_tag}_mrpFeedback"

        if sat_cfg.attitude and sat_cfg.attitude.control and sat_cfg.attitude.control.mrp_feedback:
            gains = sat_cfg.attitude.control.mrp_feedback
            mrp_ctrl.K = gains.k
            mrp_ctrl.P = gains.p
            mrp_ctrl.Ki = gains.ki
            integral_limit = gains.compute_integral_limit()
            mrp_ctrl.integralLimit = integral_limit

        return mrp_ctrl

    def _add_reaction_wheels(self, sc_sim, task_name, sc_obj, sat_cfg):
        """Add reaction wheel state effector to spacecraft.

        Uses ``simIncludeRW.rwFactory`` to create each RW device and populate
        the ``reactionWheelStateEffector`` module.

        Returns:
            Tuple of (rw_state_effector, rw_speed_out_msg) or (None, None).
        """
        rw_cfg = sat_cfg.effectors.reaction_wheels
        if not rw_cfg or not rw_cfg.wheels:
            return None, None

        # Map jitter_model string to Basilisk RWModel enum
        rw_model_map = {
            "none": messaging.BalancedWheels,
            "constant": messaging.JitterSimple,
            "speedDependent": messaging.JitterFullyCoupled,
        }

        rw_factory = simIncludeRW.rwFactory()

        for i, wheel in enumerate(rw_cfg.wheels):
            var_rw_model = rw_model_map.get(wheel.jitter_model, messaging.BalancedWheels)
            max_momentum = rw_cfg.inertia_rw * wheel.max_speed  # [Nm*s]
            label = f"RW{i}"

            rw_factory.create(
                "Custom_RW",
                wheel.direction_B,
                maxMomentum=max_momentum,
                Omega=0.0,
                RWModel=var_rw_model,
                label=label,
            )

        rw_state_eff = reactionWheelStateEffector.ReactionWheelStateEffector()
        rw_state_eff.ModelTag = f"{sat_cfg.name}_rwStateEff"
        rw_factory.addToSpacecraft(rw_state_eff.ModelTag, rw_state_eff, sc_obj)
        sc_sim.AddModelToTask(task_name, rw_state_eff)

        # Store factory so _create_rw_config_message can use it
        self._rw_factory = rw_factory

        return rw_state_eff, rw_state_eff.rwSpeedOutMsg

    def _create_rw_config_message(self, sat_cfg, model_tag):
        """Create the RW flight software configuration message.

        Delegates to the rwFactory created in _add_reaction_wheels so that
        simulation and FSW RW parameters stay in sync.
        """
        return self._rw_factory.getConfigMessage()

    def _get_recorder_sample_time(self, recorder_cfg, default_num_points: int = 1000):
        """Compute sample time for a RecorderConfig.

        If ``sample_time`` is explicitly set, return it.  Otherwise auto-
        compute a value that yields an even distribution of points over the
        simulation duration.  Returns ``None`` when the recorder is disabled.
        """
        if not recorder_cfg.enabled:
            return None
        if recorder_cfg.sample_time is not None:
            return recorder_cfg.sample_time
        sim_time = macros.min2nano(self.config.duration)
        sim_step = macros.sec2nano(self.config.dt_dynamics)
        return simHelpers.samplingTime(sim_time, sim_step, default_num_points)

    def _setup_logging(self, sc_sim, task_name, sc_obj, mrp_ctrl, rw_state_eff, sat_cfg):
        """Configure data loggers using per-config recorder settings.

        Each recordable component (state, torque, RW speeds) carries its own
        ``RecorderConfig`` with independent ``enabled`` and ``sample_time``.
        """
        # -- State recorder --
        sample_time = self._get_recorder_sample_time(sat_cfg.recorder)
        if sample_time is not None:
            sc_log = sc_obj.scStateOutMsg.recorder(int(sample_time))
            sc_sim.AddModelToTask(task_name, sc_log)

        # -- Control torque recorder --
        if sat_cfg.attitude is not None and sat_cfg.attitude.control is not None:
            ctrl = sat_cfg.attitude.control
            sample_time = self._get_recorder_sample_time(ctrl.recorder)
            if sample_time is not None:
                torque_log = mrp_ctrl.cmdTorqueOutMsg.recorder(int(sample_time))
                sc_sim.AddModelToTask(task_name, torque_log)

        # -- RW speed recorder --
        if rw_state_eff is not None and sat_cfg.effectors.reaction_wheels:
            rw_set = sat_cfg.effectors.reaction_wheels
            sample_time = self._get_recorder_sample_time(rw_set.recorder)
            if sample_time is not None:
                rw_log = rw_state_eff.rwSpeedDataMsg.recorder(int(sample_time))
                sc_sim.AddModelToTask(task_name, rw_log)

    def _configure_stop_time(self, sim):
        """Set the simulation stop time."""
        sim_time = macros.min2nano(self.config.duration)
        sim.ConfigureStopTime(sim_time)

    def _enable_visualization(self, sim):
        """Enable Unity visualization if available.

        Reads ``visualization`` section from the simulation config to
        determine ``live_stream`` and ``broadcast_stream`` flags.
        If Viz/Unity is not installed, this is a no-op.
        """
        if not vizSupport.vizFound:
            return

        viz_cfg = self.config.visualization
        live_stream = False
        broadcast_stream = False
        save_file = None

        if viz_cfg is not None:
            live_stream = viz_cfg.live_stream
            broadcast_stream = viz_cfg.broadcast_stream
            save_file = viz_cfg.save_file

        # Build kwargs for enableUnityVisualization
        kwargs = {
            "liveStream": live_stream,
            "broadcastStream": broadcast_stream,
        }
        if save_file is not None:
            kwargs["saveFile"] = save_file

        # Determine task name (use simTaskName attribute on SimBaseClass)
        task_name = self._sim_task_name or sim.simTaskName

        # Call enableUnityVisualization with spacecraft list
        vizSupport.enableUnityVisualization(
            sim, task_name, self._sc_objects, **kwargs
        )

    def _collect_results(self, sim):
        """Collect logged data into a result container."""
        satellite_results = []
        for sat_cfg in self.config.satellites:
            satellite_results.append({"name": sat_cfg.name})
        return SimulationResult(sim, satellite_results)


def run_simulation(config_path: Union[str, Path]) -> SimulationResult:
    """Convenience function to load and run a simulation from a TOML file.

    Combines :func:`load_simulation_config` and
    :class:`SimulationRunner` into a single call.

    Visualization is controlled via the ``[simulation.visualization]``
    section in the TOML config file.

    Args:
        config_path: Path to the TOML configuration file.

    Returns:
        SimulationResult with the executed simulation and satellite data.

    Example::

        from bsk_toml_sim import run_simulation

        result = run_simulation("my_sim.toml")
    """
    from bsk_toml_sim.parser import load_simulation_config

    config = load_simulation_config(config_path)
    runner = SimulationRunner(config)
    return runner.run()
