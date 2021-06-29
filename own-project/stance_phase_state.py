import math
from typing import Tuple, Any

import numpy as np
import rbdl

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

    def __init__(self, math_model: MathModel, constraint, plotter, event):
        super().__init__(math_model, constraint, plotter, event)

        self.old_t = -0.001
        self.old_J_s = np.array([[1, 1, 1, 1, ], [1, 1, 1, 1, ]], dtype=float)
        self.old_J_cog = np.array([[1, 1, 1, 1, ], [1, 1, 1, 1, ]], dtype=float)
        self.J_cog_grad = np.zeros(self.old_J_cog.shape)
        self.J_s_grad = np.zeros(self.old_J_s.shape)

    def controller_iteration(self, time, state):
        model = self.math_model.model
        q = state.q
        qd = state.qd
        gravity = model.gravity

        # Js fußpunkt 2ter link -> point ist fußspitze (die in kontakt mit boden ist)
        J_s = np.zeros((3, model.q_size))

        footpos = rbdl.CalcBodyToBaseCoordinates(model, q, model.GetBodyId("foot"), np.zeros(3), False)
        rbdl.CalcPointJacobian(model, q, model.GetBodyId("foot"), np.zeros(3), J_s, False)

        J_s = np.delete(J_s, 2, 0)

        # mass matrix
        M = np.zeros((model.q_size, model.q_size))
        rbdl.CompositeRigidBodyAlgorithm(model, q, M, False)
        M_inv = np.linalg.inv(M)

        # actuation matrix S -> only last joint is actuated
        S = np.zeros((2, model.q_size))
        S[0, 2] = 1
        S[1, 3] = 1

        # Jcog ist gesamtes (vom roboter)
        pos_com = state.pos_com()

        if pos_com is None:
            pos_com = self.com_heights[-1]

        J_com = state.jac_com()

        # b coriolis/centrifugal -> TODO: b is a generalized force -> why do we need to peoject it to the COG?
        b = np.zeros(model.q_size)
        rbdl.NonlinearEffects(model, q, qd, b)

        lambda_s = np.linalg.inv(J_s @ M_inv @ J_s.T)

        N_s = np.eye(4) - M_inv @ J_s.T @ lambda_s @ J_s

        J_star = J_com @ M_inv @ (S @ N_s).T @ np.linalg.inv(
            S @ N_s @ M_inv @ (S @ N_s).T)  # TODO: check why pinv needed -> S was the problem!
        J_star = J_star[0:2]

        lambda_star = np.linalg.pinv(J_com[0:2] @ M_inv @ (S @ N_s).T @ J_star.T)
        # lambda_star = np.linalg.inv(J_s @ M_inv @ S @ N_s @ J_s.T) # problem with dimensions

        # calculate derivative of Jacobians
        if time - self.old_t > 0.01:
            self.J_cog_grad = (1 / (time - self.old_t)) * (J_com[0:2] - self.old_J_cog[0:2])
            self.J_s_grad = (1 / (time - self.old_t)) * (J_s - self.old_J_s)

        self.old_t = time
        self.old_J_cog = J_com
        self.old_J_s = J_s

        term1 = lambda_star @ J_com[0:2] @ M_inv @ N_s.T @ b
        term2 = lambda_star @ self.J_cog_grad @ qd
        term3 = lambda_star @ J_com[0:2] @ M_inv @ J_s.T @ lambda_s @ self.J_s_grad @ state.qd
        coriolis_grav_part = term1 - term2 + term3

        # compute distance foot and COM
        distance_foot_com = pos_com - footpos
        slip_new_length = np.linalg.norm(distance_foot_com, ord=2)
        angle = np.arctan2(distance_foot_com[1], distance_foot_com[0])

        deltaX = self.math_model.slip_length - slip_new_length

        if deltaX < 0:
            deltaX = 0  # do not set negative forces -> energy loss
            self.math_model.ff = np.zeros(3)
            slip_force = np.zeros(2)
        else:
            slip_force = self.math_model.slip_stiffness * (
                np.array([deltaX * math.cos(angle), deltaX * np.sin(angle), 0])) + self.math_model.robot_mass * gravity
            self.math_model.ff = slip_force
            slip_force = slip_force[0:2]

        # Control law
        torque_new = J_star.T @ (lambda_star @ ((1 / self.math_model.robot_mass) * slip_force))
        coriolis = J_star.T @ coriolis_grav_part

        tau = np.zeros(8)
        tau[2:4] = coriolis + torque_new

        return tau

    def controller_iteration_old(self, iteration_counter: int, time, timestep: float):
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
