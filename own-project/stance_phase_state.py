import numpy as np
import rbdl

from math_model import MathModel
from motion_state_machine import State


class StancePhaseState(State):

    def run_solver(self, iteration_counter: int, timestep: float, math_model: MathModel):
        """
        run the solver for this state
        :param iteration_counter: the current iteration number
        :param timestep: length of a time step / iteration
        :param math_model: reference to the math model, where all the variables of the mathematical model
            of the robot are stored
        :return:
        """

        if not math_model.impact:  # impact of the foot / start of stance phase
            math_model.impact_com = math_model.center_of_mass
            math_model.impact = True
            math_model.first_iteration_after_impact = True
            math_model.vel_com_start_stance = math_model.vel_com

            # Energy compensation for impact at landing (kinetic energy loss of the cog point in the collision)
            mass = math_model.robot_mass
            qd = math_model.state.qd
            jac_cog = math_model.jac_cog
            nullspace_s = math_model.nullspace_s
            matrix_diff = jac_cog.transpose() @ jac_cog - nullspace_s.transpose() @ jac_cog.transpose() @ jac_cog @ nullspace_s
            vel_com_diff = qd.transpose() @ matrix_diff @ qd
            delta_e_kin = abs(0.5 * math_model.robot_mass * vel_com_diff)

            math_model.leg_length_delta = np.sqrt(2 * delta_e_kin / math_model.spring_stiffness)

            math_model.update()

        # new generalized velocity after impact
        math_model.state.qd = nullspace_s @ math_model.state.qd

        q = math_model.state.q
        state = np.concatenate((q, qd), axis=0)
        # TODO solverStandPhase.integrate(state, dt);



