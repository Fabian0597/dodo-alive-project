import math

import numpy as np
from numpy.linalg import inv
import rbdl

from articulated_leg_walker import WalkerState, State


def calc_numerical_gradient(x_new, x_old, step_size):
    return (x_new - x_old) / step_size


class MathModel:

    def __init__(self, model, walker_state: WalkerState, timestep):
        self.state = State()
        self.timestep = timestep

        self.model = model
        self.robot_mass = None
        self.jac_cog = None  # jacobianCog
        self.jac_d_cog = None  # derivative of jacobian
        self.jac_s = None
        self.jac_base_s = None
        self.jac_s_dot = None
        self.jac_star = None
        self.pos_com = None  # actualCom
        self.leg_length = None
        self.leg_angle = None
        self.nullspace_s = None
        self.lambda_s = None
        self.lambda_star = None
        self.mu_star = None
        self.p_star = None
        self.mass_matrix = None
        self.mass_matrix_ee = None
        self.b_vector = None
        self.selection_matrix = np.array([[0, 0, 1, 0], [0, 0, 0, 1]])
        self.g = np.array([0, 9.81]).transpose()

        # flight_phase TODO necessary to be in model
        self.leg_spring_delta = None  # = springLegDelta
        self.leg_length_delta = 0  # = deltaLegLength TODO what is the difference

        self.vel_com = None  # velocity of the center of gravity

        self.vel_com_start_stance = np.zeros((2, 1))  # velocity at start of stance / when the impact happens
        self.angle_of_attack = None  # in deg

        # necessary
        self.impact = None
        self.first_iteration_after_impact = True
        self.spring_stiffness = 6900

        # TODO: what is this for?
        self.impact_com = None

    def center_of_gravity_update(self):
        """
        Updates the center of gravity/mass
        """
        num_of_joints = len(self.model.mBodies)
        cog_mass_weighted = np.zeros((3, 1))
        mass_sum = 0
        for i in range(num_of_joints):
            cog_in_body = self.model.mBodies[i].mCenterOfMass
            cog_in_world = rbdl.CalcBodyToBaseCoordinates(self.model, self.state.q, i, cog_in_body, True)
            cog_mass_weighted += cog_in_world * self.model.mBodies[i].mMass
            mass_sum = mass_sum + self.model.mBodies[i].mMass

        self.pos_com = cog_mass_weighted / mass_sum

    def current_leg_length_update(self):
        """
        Updates the leg length
        """
        foot_id = self.model.GetBodyId("foot")
        pos_foot = rbdl.CalcBodyToBaseCoordinates(self.model, self.state.q, foot_id, np.zeros(3), True)
        self.leg_length = np.linalg.norm(self.pos_com - pos_foot, ord=2)

    def current_leg_angle_update(self):
        """
        Updates the leg angle
        """
        foot_id = self.model.GetBodyId("foot")
        pos_foot = rbdl.CalcBodyToBaseCoordinates(self.model, self.state.q, foot_id, np.zeros(3), True)
        dx = pos_foot[0] - self.pos_com[0]
        dy = pos_foot[1] - self.pos_com[1]
        self.leg_angle = math.degrees(math.atan2(abs(dx), abs(dy)))

    def jacobian_cog_update(self):
        """
        Updates the Jacobian (center of gravity)
        """
        num_of_joints = len(self.model.mBodies)
        # TODO numerical? previous_jac = self.jac_cog
        jac_cog_sum = np.zeros((3, self.model.dof_count))
        mass_sum = 0

        for i in range(num_of_joints):
            jac_cog_i = np.zeros((3, self.model.dof_count))
            body_i = self.model.GetBody(i)
            rbdl.CalcPointJacobian(self.model, self.state.q, i, body_i.mCenterOfMass, jac_cog_i, True)
            jac_cog_i = jac_cog_i * body_i.mMass
            mass_sum += body_i.mMass
            jac_cog_sum += jac_cog_i

        self.jac_cog = jac_cog_sum / mass_sum
        self.robot_mass = mass_sum

        jac_dot_cog = np.zeros((3, self.model.dof_count))
        floatingbase_id = self.model.GetBodyId("floatingBase")
        rbdl.CalcPointJacobian(self.model, self.state.q, floatingbase_id, np.zeros(3), jac_dot_cog, True)
        self.jac_d_cog = jac_dot_cog
        # TODO: numerical? jacobianCogDot = calc_gradient(jacobianCog,jacobianCogOld,timeStep);

    def jacobian_s_update(self):
        jac_base = np.zeros((6, self.model.dof_count))
        jac_foot = np.zeros((6, self.model.dof_count))

        pre_jac_s = self.jac_s

        rbdl.CalcPointJacobian(self.model, self.state.q, self.model.GetBodyId("floatingBase"), np.zeros(3), jac_base,
                               True)
        rbdl.CalcPointJacobian(self.model, self.state.q, self.model.GetBodyId("foot"), np.zeros(3), jac_foot, True)
        self.jac_base_s = jac_foot - jac_base
        self.jac_s = jac_foot
        self.jac_s_dot = calc_numerical_gradient(self.jac_s, pre_jac_s, self.timestep)

    def jacobian_star_update(self):
        """
        update the nullspace_s in Hutter paper (8)
        """
        actuation_matrix_nullspace_s = self.selection_matrix * self.nullspace_s
        self.jac_star = self.jac_cog * inv(self.mass_matrix) * actuation_matrix_nullspace_s.transpose() * inv(
            actuation_matrix_nullspace_s * inv(self.mass_matrix) * actuation_matrix_nullspace_s.transpose())

    def nullspace_s_update(self):
        """
        update the nullspace_s Hutter paper between (4) and (5)
        """
        mass_matrix_jac_S_lambda_s_jac_S = inv(self.mass_matrix) * self.jac_s.transpose() * self.lambda_s * self.jac_s
        identity_matrix = np.identity(np.shape(mass_matrix_jac_S_lambda_s_jac_S[0]))
        self.nullspace_s = identity_matrix - mass_matrix_jac_S_lambda_s_jac_S

    def lambda_s_update(self):
        """
        update the inertia matrix s Hutter paper between (4) and (5)
        """
        self.lambda_s = inv(self.jac_s * inv(self.mass_matrix) * self.jac_s.transpose())

    def lambda_star_update(self):
        """
        update the inertia matrix star Hutter paper (9)
        """
        # TODO: what is the normal J equal to Jcog like in C++ implementation?
        actuation_matrix_nullspace_s = self.selection_matrix * self.nullspace_s
        self.lambda_star = inv(self.jac_cog * inv(self.mass_matrix) * actuation_matrix_nullspace_s * self.jac_star)

    def mu_star_update(self):
        """
        update the mue star Hutter paper (10)
        """
        # TODO self.qd, self.b_vector is still missing
        mue_star_1 = self.lambda_star * self.jac_cog * inv(
            self.mass_matrix) * self.nullspace_s.transpose() * self.b_vector
        mue_star_2 = self.lambda_star * self.jac_d_cog * self.state.qd
        mue_star_3 = self.lambda_star * self.jac_cog * inv(
            self.mass_matrix) * self.jac_s.transpose() * self.lambda_s * self.jac_s_dot * self.state.qd
        self.mu_star = mue_star_1 - mue_star_2 + mue_star_3

    def p_star_update(self):
        """
        update the p star Hutter paper (11)
        """
        self.p_star = self.lambda_star * self.jac_cog * inv(self.mass_matrix) * self.nullspace_s.transpose() * self.g

    def mass_matrix_update(self):
        """
        Computes the joint space inertia matrix by using the Composite Rigid Body Algorithm
        """
        rbdl.CompositeRigidBodyAlgorithm(self.model, self.state.q, self.mass_matrix, True)

    def b_vector_update(self):
        """
        calculates the vector b which acounts for coriolis and centrifugal force
        InverseDynamics: This function computes the generalized forces (actuations of the internal joints (output)) from given generalized states,
            velocities, and accelerations. Computes inverse dynamics with the Newton-Euler Algorithm
        NonlinearEffects: Computes the coriolis forces
        """
        rbdl.NonlinearEffects(self.model, self.state.q, self.state.qd, self.b_vector)
        pass

    # def com_vel(self):
    #    jac = self.jac_cog()
    #    return jac @ self.qd

    def update(self):
        pass
