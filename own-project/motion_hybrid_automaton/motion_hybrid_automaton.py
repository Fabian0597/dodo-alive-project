import math
from enum import Enum

import numpy as np
import rbdl

from scipy.integrate import solve_ivp

from motion_hybrid_automaton.guard_functions import GuardFunctions
from motion_hybrid_automaton.jump_functions import JumpFunctions

"""
import sys
import pathlib
basefolder = str(pathlib.Path(__file__).parent.absolute())
sys.path.append(basefolder+'/../../rbdl-tum/build/python/')
import rbdl
"""

from model.slip_model_params import SlipModelParameterization
from motion_hybrid_automaton.phases.flight_phase_state import FlightPhaseState
from motion_hybrid_automaton.phases.stance_phase_state import StancePhaseState

import logging


class DiscreteState(Enum):
    STANCE = 0
    FLIGHT = 1
    NOT_MOVING = 2


class MotionHybridAutomaton:
    """
    This hybrid automaton/state machine represents the states of the walker and the transitions to switch between states.
    The state consists of a discrete state (z) and a continuous state (x).
    z: whether it is in stance or fight mode
    x: position / velocity
    """

    def __init__(self, leg_model, des_com_pos, constraint_flight, constraint_stance, gui_plot):
        logging.info("Init Motion Hybrid Automaton")
        logging.debug("Create and update math model")
        self.model = leg_model
        self.slip_model = SlipModelParameterization()
        self.gui_plot = gui_plot

        guard = GuardFunctions(self.model, self.slip_model)
        jump = JumpFunctions(self)

        logging.debug("Create State Phases")
        self.states = {
            DiscreteState.FLIGHT: FlightPhaseState(self, constraint_flight,
                                                   [guard.flight_to_stance_guard_function()]),
            DiscreteState.STANCE: StancePhaseState(self, constraint_stance,
                                                   [guard.stance_to_flight_guard_function()]),
        }

        logging.debug("Create Transitions")
        self.transitions = {
            DiscreteState.FLIGHT: [DiscreteState.STANCE, jump.flight_to_stance_jump_function],
            DiscreteState.STANCE: [DiscreteState.FLIGHT, jump.stance_to_flight_jump_function],
        }

        self.z = DiscreteState.FLIGHT  # discrete state
        # self.x = init_state  # continuous state

    def current_constraint(self):
        return self.states[self.z].constraint

    def simulate_motion(self, t_final, init_state, log_callback):
        """
        Calculates the differential equation of the movement for the stance and flight phase in sections alternately
        and logs the movement so that it can be visualised later in MeshUp.
        :param t_final: time when the solver should stop calculating the motion
        :param init_state: the initial state of the robot leg at time (t=0)
        :param log_callback: this method is called for logging a single time point with corresponding state
        """
        time, x = 0, init_state
        logging.info("Start motion simulation")

        while time < t_final:
            # Solve an initial value problem for a system of ordinary differential equations (ode)
            # Interval of integration (t_init,, t_final).
            # state = initial state
            # function to be integrated = f
            # it solves the ode until it reaches the event touchdown_event(y,t)=0
            # so the foot hits the ground and has (y height 0)
            active_state = self.states[self.z]
            # TODO: use t_eval ?
            # np.arange(time, t_final, 0.05)
            # t_eval = np.array([0.4, 0.7]).copy(order='C')
            # print(t_eval)
            solver = solve_ivp(
                fun=active_state.flow_function,
                t_span=[time, t_final], y0=x,
                max_step=0.2,
                events=active_state.events
            )
            time, x = solver.t[-1], solver.y.T[-1]

            # iterate over internal states saved by the solver during integration
            for i in range(0, len(solver.t)):
                # returns the value of the solution y=[q,qd] at time stamp T[i] from the interval of integration
                time = solver.t[i]
                x_t = solver.y.T[i][:self.model.dof_count]  # state at time T[i] without derivatives
                log_callback(time, x_t)

            # transition to next discrete state
            # (continuous state x is processed in the jump function to enter new discrete state)
            self.z, jump_function = self.transitions[self.z]
            time, x = jump_function(time, x)
