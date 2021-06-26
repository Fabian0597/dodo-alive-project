from typing import Tuple, Any

import numpy as np

"""
import sys
import pathlib
basefolder = str(pathlib.Path(__file__).parent.absolute())
sys.path.append(basefolder+'/../../rbdl-tum/build/python/')
import rbdl
"""

from math_model import MathModel
from phase_state import PhaseState


class StancePhaseState(PhaseState):

    def __init__(self, math_model: MathModel, constraint):
        super().__init__(math_model, constraint)

        def jumping_event(time, y):
            """
            This event marks the transition from stance phase to fight phase, when the leg not touches the ground any more.
            It is an event function for integrating solver of the stance phase.
            When touchdown_event(t,y) = 0 solver stops
            :param time: time
            :param y: generalized coordinates and their derivative y=[q, qd].transpose()
            :return:
            """
            # from c++ code: foot_pos[1] < 0 and not (springLegDelta < 0.0 && vel_cog(1) > 0.0)
            pass

        jumping_event.terminal = True

        self.events = [jumping_event]

    def controller_iteration(self, iteration_counter: int, timestep: float):
        """
        performs iteration for the controller of the stance phase.
        This simulates a mass-spring system (SLIP)
        :param iteration_counter: iteration for counter of the solver
        :param timestep: timestep of the current iteration
        :param math_model: reference to the math_model
        :return: torque (tau) from the controller (state of the robot's leg)
        """

        if not self.math_model.impact:  # impact of the foot / start of stance phase
            self.math_model.impact_com = self.math_model.pos_com
            self.math_model.impact = True
            self.math_model.first_iteration_after_impact = True
            self.math_model.vel_com_start_stance = self.math_model.vel_com #where do we need that?

            # Energy compensation for impact at landing (kinetic energy loss of the cog point in the collision)
            mass = self.math_model.robot_mass
            qd = self.math_model.state.qd
            jac_cog = self.math_model.jac_cog
            nullspace_s = self.math_model.nullspace_s
            matrix_diff = jac_cog.transpose() @ jac_cog - nullspace_s.transpose() @ jac_cog.transpose() @ jac_cog @ nullspace_s
            vel_com_diff = qd.transpose() @ matrix_diff @ qd
            delta_e_kin = abs(0.5 * mass * vel_com_diff)

            self.math_model.leg_length_delta = np.sqrt(2 * delta_e_kin / self.math_model.spring_stiffness)

            self.math_model.update()

        # new generalized velocity after impact
        self.math_model.state.qd = self.math_model.nullspace_s @ self.math_model.state.qd

        tau_stance = self.math_model.jac_star.transpose() @ self.math_model.spaceControlForce

        print("tau_stance: %s" % tau_stance.flatten())  # why is this matrix?

        return tau_stance.flatten()

    def transfer_to_next_state(self, solver_result) -> Tuple[Any, Any]:
        """
        transfer stance to flight
        :param solver_result: resulting output of the solver
        :return: a tuple of robot's state for the next phase and the end time (start time for the next state)
        """
        t_init = solver_result.t[-1]
        solver_state = solver_result.y.T[-1]
        return solver_state, t_init
