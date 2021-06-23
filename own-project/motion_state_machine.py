from typing import Tuple, Any

import numpy as np
import sys
import pathlib

from scipy.integrate import solve_ivp

import articulated_leg_walker
import math_model
from articulated_leg_walker import State

basefolder = str(pathlib.Path(__file__).parent.absolute())
sys.path.append(basefolder + '/../../rbdl-orb/build/python/')
import rbdl

from math_model import MathModel
from flight_phase_state import FlightPhaseState
from stance_phase_state import StancePhaseState


class State:

    def __init__(self, math_model: MathModel):
        self.iteration_counter = 0
        self.last_timestep = None
        self.events = None
        self.math_model = math_model

    def controller_iteration(self, iteration_counter: int, timestep: float):
        """
        performs iteration for a time step for the controller of this state
        :param iteration_counter: iteration for counter of the solver
        :param timestep: timestep of the current iteration
        :return: torque (tau) from the controller (state of the robot's leg)
        """
        pass  # this is implemented for the concrete states only

    def transfer_to_next_state(self, solver_result) -> Tuple[Any, Any]:
        """
        this function maps the output of the solver of this state to the information for the next state
        :param solver_result: resulting output of the solver
        :return: a tuple of robot's state for the next phase and the end time (start time for the next state)
        """
        pass  # this is implemented for the concrete states only

    def solver_function(self, time, y):
        """
        computes the generalized accelerations from given generalized states, velocities and forces during flight
        :param time: time
        :param y: generalized coordinates and their derivatives y=[q, qd].transpose()
        :return: derivative yd = [qd, qdd]
        """
        self.iteration_counter += 1
        delta_time = self._get_delta_time(time)

        self.math_model.get_timestep(delta_time)

        self.math_model.state = State.from_q_qd_array(y, self.dof_count)

        tau_desired = self.controller_iteration(self.iteration_counter, delta_time)

        state_derivative = self._forward_dynamics(tau_desired)

        return state_derivative

    def _forward_dynamics(self, tau_desired):
        """
        calculates the forward dynamics given tau from the controller, f_ext and the leg state q, qd
        :param leg_state: state variables of the leg (q, qd, tau)
        :return: qd, qdd
        """
        # calculate qdd from q, qd, tau, f_ext for the model with the forward dynamics
        # rbdl.ForwardDynamicsConstraintsDirect(self.leg_model, state.q, state.qd, state.tau,
        #   self.constraints[self.discrete_state], state.qdd) --> this is just done during stance
        #rbdl.ForwardDynamics(self.math_model.model, leg_state.q, leg_state.qd, leg_state.tau, leg_state.qdd)
        rbdl.ForwardDynamics(self.math_model.model, self.math_model.state.q, self.math_model.state.qd, tau_desired, self.math_model.state.qdd)

        # return res = [qd, qdd] from the forward dynamics which can then be given to the solver to be integrated by the ivp_solver
        state = np.zeros(2 * self.dof_count)
        state[:self.dof_count] = self.math_model.state.qd
        state[self.dof_count:] = self.math_model.state.qdd
        return state

    def _get_delta_time(self, time):
        if not self.last_timestep:
            delta_time = 1e-12  # 0
        else:
            delta_time = time - self.last_timestep
        self.last_timestep = time
        return delta_time


class MotionStateMachine:
    """
    This state machine represents the state of the walker,
    whether it is in stance or fight mode
    """

    def __init__(self, leg_model, des_com_pos):
        self.math_model = MathModel(leg_model, des_com_pos)
        flight_state = FlightPhaseState(self.math_model)
        stance_state = StancePhaseState(self.math_model)
        self.states = {}
        self.states["Flight"] = flight_state
        self.states["Stance"] = stance_state
        self.active_state: State = self.states.get("Flight")

        self.transitions = {}
        self.transitions[flight_state] = stance_state
        self.transitions[stance_state] = flight_state

    def switch_to_next_state(self, solver_result):
        """
        Switches to the next state
        :param solver_result: solver result of the last state solver iteration
        :return: robot's state and time of the end of the last phase / state
        """
        state, time = self.active_state.transfer_to_next_state(solver_result)
        self.active_state = self.transitions[self.active_state]

        # give new state to active_state
        self.active_state.math_model.state = self.active_state.math_model.state.from_q_qd_array(state, self.dof_count)
        return state, time
