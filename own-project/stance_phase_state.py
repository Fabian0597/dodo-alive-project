import numpy as np
import rbdl

from dynamics import MathModel
from motion_state_machine import State


class StancePhaseState(State):

    def run_solver(self, math_model: MathModel):
        """
        run the solver for this state
        """
