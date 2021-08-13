import math

import numpy as np
import rbdl

from motion_hybrid_automaton.continuous_state import ContinuousState

"""
import sys
import pathlib
basefolder = str(pathlib.Path(__file__).parent.absolute())
sys.path.append(basefolder+'/../../rbdl-tum/build/python/')
import rbdl
"""

from motion_hybrid_automaton.phases.abstract_phase_state import AbstractPhaseState


class StancePhaseState(AbstractPhaseState):
    """
    controller during flight phase
    """

    def __init__(self, hybrid_automaton, constraint, guard_functions):
        super().__init__(hybrid_automaton, constraint, guard_functions)

        self.old_J_s = np.array([[1, 1, 1, 1, ], [1, 1, 1, 1, ]], dtype=float)  # old J_s for numerical gradient
        self.J_s_grad = np.zeros(self.old_J_s.shape)  # numerical gradient for J_s

        self.old_J_com = np.array([[1, 1, 1, 1, ], [1, 1, 1, 1, ]], dtype=float)  # old J_com for numerical gradient
        self.J_com_grad = np.zeros(self.old_J_com.shape)  # numerical gradient of J_com

    def controller_iteration(self, time, state: ContinuousState):
        """
        performs iteration for the controller of the stance phase.
        SLIP motion control law for any arbitrary robotic leg
        :param time: current time
        :param state: robot state
        :return: tau to control the robot torques
        """
        model = self.model
        q = state.q
        qd = state.qd
        gravity = model.gravity

        # position of foot
        footpos = rbdl.CalcBodyToBaseCoordinates(model, q, model.GetBodyId("foot"), np.zeros(3), False)

        # Jacobian in foot frame from the function defined in the ContinuousState Class
        # (TODO: in old skript jac_s was Jacobian_foot-Jacobian_base)
        jac_s = state.jac_s()

        # inverse mass matrix from the function defined in the ContinuousState Class
        inv_mass_matrix = state.inv_mass_matrix()

        # actuation matrix S (tau_1 = q_3, tau_2 = q_4)
        S = np.zeros((2, model.q_size))
        S[0, 2] = 1
        S[1, 3] = 1

        # Calculated position of the COM from the function defined in the ContinuousState Class
        pos_com = state.pos_com()

        # Calculated robot mass from the function defined in the ContinuousState Class
        robot_mass = state.robot_mass()

        # Calculated the Jaobian in the COM frame from the function defined in the ContinuousState Class
        J_com = state.jac_com()[0:2]

        # calculates the vector b which acounts for coriolis and centrifugal force
        # -> TODO: b is a generalized force -> why do we need to project it to the COG?
        b = np.zeros(model.q_size)
        rbdl.NonlinearEffects(model, q, qd, b)

        # update the inertia matrix s Hutter paper between(4) and (5)
        lambda_s = np.linalg.inv(jac_s @ inv_mass_matrix @ jac_s.T)

        # update the nullspace_s Hutter paper between (4) and (5)
        N_s = np.eye(4) - inv_mass_matrix @ jac_s.T @ lambda_s @ jac_s

        # update the nullspace_s in Hutter paper (8)
        J_star = J_com @ inv_mass_matrix @ (S @ N_s).T @ np.linalg.inv(S @ N_s @ inv_mass_matrix @ (
                    S @ N_s).T)  # TODO: check why pinv needed -> S was the problem! --> where was the pinv ? (question by Fabian)
        J_star = J_star[0:2]

        # update the inertia matrix s Hutter paper between (4) and (5)
        # (changed to normal inv since it we also did that in the other code and it works here as well (Fabian)
        lambda_star = np.linalg.pinv(J_com @ inv_mass_matrix @ (S @ N_s).T @ J_star.T)
        # lambda_star = np.linalg.inv(jac_s @ inv_mass_matrix @ S @ N_s @ jac_s.T) # problem with dimensions

        # calculate derivative of Jacobian s in foot frame and Jacobian com in com frame
        time_diff = time - self.last_iteration_time
        if time_diff > 1e-4:
            self.J_com_grad = self.calc_numerical_gradient(self.old_J_com, J_com, time_diff)
            self.J_s_grad = self.calc_numerical_gradient(self.old_J_s, jac_s, time_diff)

        # update old Jacobian s in foot frame and Jacobian com in com frame
        # and time for calculating the derivative in the next iteration
        self.old_J_com = J_com
        self.old_J_s = jac_s

        # coriolis and centrifugal part in tau calculation (mu_star) star Hutter paper (10)
        term1 = lambda_star @ J_com @ inv_mass_matrix @ N_s.T @ b
        term2 = lambda_star @ self.J_com_grad @ qd
        term3 = lambda_star @ J_com @ inv_mass_matrix @ jac_s.T @ lambda_s @ self.J_s_grad @ state.qd
        coriolis_part = term1 - term2 + term3

        # gravitational part in tau calculation (p_star) star Hutter paper (11)
        grav_part = lambda_star @ J_com @ inv_mass_matrix @ N_s.T @ np.array(
            [0, 0, 9.81, 0])  # TODO use gravity variable

        # p_star and mu star added Hutter paper (12)
        coriolis_grav_part = coriolis_part + grav_part
        # compute distance and orientation of foot and COM (which will simulate the spring length and orientation)
        distance_foot_com = pos_com - footpos
        slip_new_length = np.linalg.norm(distance_foot_com, ord=2)
        angle = np.arctan2(distance_foot_com[1], distance_foot_com[0])

        # compression of the foot calculated with l_0 length of leg and current length of leg
        spring_compression = self.slip_model.slip_length - slip_new_length

        # calculate slip force from spring stiffness and spring compression,
        # separate in x, y direction to keep force direction
        spring_compression_vec = np.array([
            spring_compression * np.cos(angle), spring_compression * np.sin(angle), 0])
        slip_force = self.slip_model.slip_stiffness * spring_compression_vec + robot_mass * gravity
        self.slip_model.slip_force = slip_force
        slip_force = slip_force[0:2]

        # Control law
        # torque applied during stance phase to generate the slip force
        torque_new = J_star.T @ (lambda_star @ ((1 / robot_mass) * slip_force))
        # torque applied during stance phase to oppose coriolis and centrifugal and gravitational force
        coriolis = J_star.T @ coriolis_grav_part

        # final torque
        tau = np.zeros(8)
        tau[2:4] = coriolis + torque_new

        return tau
