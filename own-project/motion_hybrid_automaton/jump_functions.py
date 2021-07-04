import logging
import math

import numpy as np
import rbdl

from motion_hybrid_automaton.continuous_state import ContinuousState


class JumpFunctions:

    def __init__(self, motion_hybrid_automaton):
        self.model = motion_hybrid_automaton.model
        self.slip_model = motion_hybrid_automaton.slip_model
        self.motion_hybrid_automaton = motion_hybrid_automaton

    def _calculate_new_slip_length(self, time, state):
        # TODO check the Force is towards ground at the beginning of stance phase
        # TODO remove unused lines
        jac_com = state.jac_com()
        robot_mass = state.robot_mass()
        jac_s = np.zeros((3, self.model.q_size))
        rbdl.CalcPointJacobian(self.model, state.q, self.model.GetBodyId("foot"), np.zeros(3), jac_s, False)
        jac_s = np.delete(jac_s, 2, 0)
        # mass matrix
        mass_matrix = np.zeros((self.model.q_size, self.model.q_size))
        rbdl.CompositeRigidBodyAlgorithm(self.model, state.q, mass_matrix, False)
        mass_matrix_inv = np.linalg.inv(mass_matrix)
        lambda_s = np.linalg.inv(jac_s @ mass_matrix_inv @ jac_s.T)
        null_s = np.eye(4) - mass_matrix_inv @ jac_s.T @ lambda_s @ jac_s
        matrix = jac_com.T @ jac_com - null_s.T @ jac_com.T @ jac_com @ null_s

        # post_vel = np.linalg.norm(jac_com @ null_s @ np.linalg.pinv(jac_com) @ state.vel_com(), ord=2)
        delta_leg_length = (1 / self.slip_model.slip_stiffness) * robot_mass * (
                state.qd.T @ matrix @ state.qd)
        delta_leg_length = math.sqrt(abs(delta_leg_length))

        foot_id = self.model.GetBodyId('foot')
        foot_pos = rbdl.CalcBodyToBaseCoordinates(self.model, state.q, foot_id, np.zeros(3), True)
        com_pos = state.pos_com()
        l0_before_impact = np.sum(com_pos - foot_pos)

        self.slip_model.slip_length = l0_before_impact + delta_leg_length
        #self.slip_model.slip_length = self.slip_model.slip_length_zero + delta_leg_length
        # energy = 0.5 * robot_mass * state.qd.T @ matrix @ state.qd

    def flight_to_stance_jump_function(self, time, x):
        # Calculation of change of velocity through impact
        # qd_minus is the generalized velocity before and qd_plus the generalized velocity after the impact
        """
        q_minus = x[:self.model.dof_count]  # first half of state vector is q
        qd_minus = x[self.model.dof_count:]  # second half of state vector is qd
        qd_plus = np.ones(self.model.dof_count)
        rbdl.ComputeConstraintImpulsesDirect(self.model, q_minus, qd_minus, self.constraint, qd_plus)
        # replace generalized velocity in state vector for next iteration by calculated velocity after ground impact
        x[self.model.dof_count:] = qd_plus
        """
        logging.info("change phase: flight > stance (time: %s)" % time)

        # take robot snapshot
        state = ContinuousState(self.model, x)
        # compute impulses
        self._calculate_new_slip_length(time, state)
        # vel = np.linalg.norm(state.com_vel(), ord=2)
        qd_plus = np.ones(self.model.dof_count)
        rbdl.ComputeConstraintImpulsesDirect(self.model, x[:self.model.dof_count], x[self.model.dof_count:],
                                             self.motion_hybrid_automaton.current_constraint(),
                                             qd_plus)
        x[self.model.dof_count:] = qd_plus
        return time, x

    def stance_to_flight_jump_function(self, time, x):
        logging.info("change phase: stance > flight (time: %s)" % time)
        return time, x