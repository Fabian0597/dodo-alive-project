import logging
import math

import numpy as np
import rbdl

from motion_hybrid_automaton.continuous_state import ContinuousState


class JumpFunctions:
    """
    defines the transition between stance and flight phase. Energy compensation from flight to stance
    """
    def __init__(self, motion_hybrid_automaton):
        self.model = motion_hybrid_automaton.model  # lua leg model
        self.slip_model = motion_hybrid_automaton.slip_model  # leg model parameter
        self.motion_hybrid_automaton = motion_hybrid_automaton  # state machine

    def _calculate_new_slip_length(self, state):
        # TODO check the Force is towards ground at the beginning of stance phase
        # TODO remove unused lines
        """
        modified  SLIP model with impact compensation which differs from the regular SLIP model only in the fact
        that the leg spring is being pre-compressed at touch-down. The precompressed leg length is calculated here.
        :param state: robot state
        """
        # get jacobians, robot mass, inverse of mass matrix from the function defined in the ContinuousState Class
        jac_com = state.jac_com()
        robot_mass = state.robot_mass()
        jac_s = state.jac_s()
        mass_matrix_inv = state.inv_mass_matrix()

        # get energy loss during impact in Hutter paper (17)
        lambda_s = np.linalg.inv(jac_s @ mass_matrix_inv @ jac_s.T)
        null_s = np.eye(4) - mass_matrix_inv @ jac_s.T @ lambda_s @ jac_s
        matrix = jac_com.T @ jac_com - null_s.T @ jac_com.T @ jac_com @ null_s
        delta_E = 0.5 * robot_mass * state.qd.T @ matrix @ state.qd

        # get compression of spring to compensate the energy loss in Hutter paper (16)
        delta_leg_length = (2 / self.slip_model.slip_stiffness) * delta_E
        delta_leg_length = math.sqrt(abs(delta_leg_length))

        # kinematic leg length at touchdown which is used as l0 leg length during impact compensation in Hutter paper block after (17)

        foot_id = self.model.GetBodyId('foot')
        foot_pos = rbdl.CalcBodyToBaseCoordinates(self.model, state.q, foot_id, np.zeros(3), True)
        pos_com = state.pos_com()
        l0_before_impact = np.linalg.norm(pos_com - foot_pos)

        # switch to a model with new leg length which compensates the impact in Hutter paper block after (17)
        self.slip_model.slip_length = l0_before_impact + delta_leg_length

    def flight_to_stance_jump_function(self, time, x):
        """
        Calculation of change of velocity through imp
        :param time: current time
        :param x: generalized velocity and vector before the impact
        :return: generalized velocity and vector after the impact and current time
        """

        logging.info("change phase: flight > stance (time: %s)" % time)

        # used object of ContinuousState to better handle the robot state
        state = ContinuousState(self.model, x)
        # compute pre-compressed leg length in order to compensate impact according to Hutter paper (16), (17)
        self._calculate_new_slip_length(state)

        # calculate the velocity after impact qd_plus
        qd_plus = np.ones(self.model.dof_count)
        rbdl.ComputeConstraintImpulsesDirect(self.model, x[:self.model.dof_count], x[self.model.dof_count:],
                                             self.motion_hybrid_automaton.current_constraint(),
                                             qd_plus)

        # update general velocity in robot state space x = [q, qd]
        x[self.model.dof_count:] = qd_plus
        return time, x

    def stance_to_flight_jump_function(self, time, x):
        """
        Nothing is calculated while transition form stance to flight phase
        :param time: current time
        :param x: generalized velocity and vector
        :return: generalized velocity and vector and current time
        """
        logging.info("change phase: stance > flight (time: %s)" % time)
        return time, x
