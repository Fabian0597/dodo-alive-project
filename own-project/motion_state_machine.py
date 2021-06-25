from typing import Tuple, Any

import numpy as np


from scipy.integrate import solve_ivp
"""
import sys
import pathlib
basefolder = str(pathlib.Path(__file__).parent.absolute())
sys.path.append(basefolder+'/../../rbdl-tum/build/python/')
import rbdl
"""

from math_model import MathModel, State
from flight_phase_state import FlightPhaseState
from stance_phase_state import StancePhaseState

import logging



class MotionStateMachine:
    """
    This state machine represents the state of the walker,
    whether it is in stance or fight mode
    """

    def __init__(self, leg_model, des_com_pos, constraint):
        logging.debug("Create and update math model")
        self.math_model = MathModel(leg_model, des_com_pos)
        logging.debug("Update MathModel")
        #self.math_model.update() #TODO: where to update math model for the first time there where errors where some values of math model were used but still initialized with NONE ("typeerror-nonetype-object-is-not-subsriptable")

        logging.debug("Create State Phases")
        flight_state = FlightPhaseState(self.math_model, constraint)
        stance_state = StancePhaseState(self.math_model)
        self.dof_count = leg_model.qdot_size
        self.states = {}
        self.states["Flight"] = flight_state
        self.states["Stance"] = stance_state
        self.active_state: State = self.states.get("Flight")

        logging.debug("Create Transitions")
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
