import math

import numpy as np
from numpy.linalg import inv

import sys
import pathlib

basefolder = str(pathlib.Path(__file__).parent.absolute())
sys.path.append(basefolder + '/../../rbdl-tum/build/python/')
import rbdl
import logging


def calc_numerical_gradient(x_new, x_old, step_size):
    if x_old is None:
        return np.zeros(np.array(x_new).shape)
    return (x_new - x_old) / step_size


class State:
    def __init__(self, q_size: int, q=None, qd=None, qdd=None):
        self.q = q or np.zeros(q_size)
        self.qd = qd or np.zeros(q_size)
        self.qdd = qdd or np.zeros(q_size)

    @classmethod
    def from_q_qd_array(cls, y, q_size):
        state = State(q_size)
        state.q = y[:q_size]
        state.qd = y[q_size:]
        return state

    def to_q_qd_array(self):
        return np.concatenate((self.q, self.qd), axis=0)

class MathModel:
    """
    represents the mathematical model, where all the variables of the robot leg are stored
    """

    def __init__(self, model, des_com_pos):
        self.state = State(model.qdot_size)

        self.model = model
        self.timestep = 1e-12
        self.des_com_pos = des_com_pos
        self.robot_mass = None
        self.jac_cog = None  # jacobianCog
        self.jac_d_cog = None  # derivative of jacobian
        self.jac_s = None
        self.jac_base_s = None
        self.jac_s_dot = None
        self.jac_star = None
        self.pos_com = None  # actualCom
        self.pos_com_old = None
        self.leg_length = None
        self.leg_angle = None
        self.nullspace_s = None
        self.lambda_s = None
        self.lambda_star = None
        self.mu_star = None
        self.p_star = None
        self.mass_matrix = np.zeros((4, 4))
        self.mass_matrix_ee = np.zeros((4, 4))
        self.b_vector = None
        self.selection_matrix = np.array([[0, 0, 1, 0], [0, 0, 0, 1]])
        self.g = np.array([0, 9.81]).transpose()
        self.springLegForce = None
        self.spaceControlForce = None

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

    def get_timestep(self, timestep):
        self.timestep = timestep

    def center_of_gravity_update(self):
        """
        Updates the center of gravity/mass
        """
        num_of_joints = len(self.model.mBodies)
        cog_mass_weighted = np.zeros((3))
        mass_sum = 0
        for i in range(num_of_joints):
            cog_in_body = self.model.mBodies[i].mCenterOfMass
            cog_in_world = rbdl.CalcBodyToBaseCoordinates(self.model, self.state.q, i, cog_in_body, True)
            cog_mass_weighted += cog_in_world * self.model.mBodies[i].mMass
            mass_sum = mass_sum + self.model.mBodies[i].mMass

        self.pos_com = (cog_mass_weighted / mass_sum)[:2]



    def vel_center_of_gravity_upate(self):
        if self.pos_com_old is not None:
            self.vel_com = self.pos_com - self.pos_com_old
        else:
            self.vel_com = self.pos_com - np.zeros((np.shape(self.pos_com)))
        self.pos_com_old = self.pos_com  # TODO ist the order of calculatin com_pos, com_vel, update com_pos_old right?

    def current_leg_length_update(self):
        """
        Updates the leg length
        """
        foot_id = self.model.GetBodyId("foot")
        pos_foot = rbdl.CalcBodyToBaseCoordinates(self.model, self.state.q, foot_id, np.zeros(3), True)[:2]
        self.leg_length = np.linalg.norm(self.pos_com - pos_foot, ord=2)

    def current_leg_angle_update(self):
        """
        Updates the leg angle
        """
        foot_id = self.model.GetBodyId("foot")
        pos_foot = rbdl.CalcBodyToBaseCoordinates(self.model, self.state.q, foot_id, np.zeros(3), True)[:2]
        dx = pos_foot[0] - self.pos_com[0]
        dy = pos_foot[1] - self.pos_com[1]
        self.leg_angle = math.degrees(math.atan2(abs(dx), abs(dy)))

    def jacobian_cog_update(self):
        """
        Updates the Jacobian (center of gravity)
        """
        num_of_joints = len(self.model.mBodies)
        # TODO numerical? previous_jac = self.jac_cog # TODO: oben wird init von jac mit 6xdof initialisiert, was íst besser?
        jac_cog_sum = np.zeros((3, self.model.dof_count))
        mass_sum = 0

        for i in range(num_of_joints):
            jac_cog_i = np.zeros((3, self.model.dof_count))
            body_i = self.model.GetBody(i)
            rbdl.CalcPointJacobian(self.model, self.state.q, i, body_i.mCenterOfMass, jac_cog_i, True)
            jac_cog_i = jac_cog_i * body_i.mMass
            mass_sum += body_i.mMass
            jac_cog_sum += jac_cog_i

        self.jac_cog = (jac_cog_sum / mass_sum)[:2,:self.model.dof_count]
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

        rbdl.CalcPointJacobian(self.model, self.state.q, self.model.GetBodyId("floatingBase"), np.zeros(3), jac_base,True)
        rbdl.CalcPointJacobian(self.model, self.state.q, self.model.GetBodyId("foot"), np.zeros(3), jac_foot, True)
        self.jac_base_s = jac_foot[:2,:self.model.dof_count] - jac_base[:2,:self.model.dof_count]
        self.jac_s = jac_foot[:2,:self.model.dof_count]
        self.jac_s_dot = calc_numerical_gradient(self.jac_s, pre_jac_s, self.timestep)

    def jacobian_star_update(self):
        """
        update the nullspace_s in Hutter paper (8)
        """
        actuation_matrix_nullspace_s = self.selection_matrix @ self.nullspace_s
        self.jac_star = self.jac_cog @ inv(self.mass_matrix) @ actuation_matrix_nullspace_s.transpose() @ inv(
            actuation_matrix_nullspace_s @ inv(self.mass_matrix) @ actuation_matrix_nullspace_s.transpose())

    def nullspace_s_update(self):
        """
        update the nullspace_s Hutter paper between (4) and (5)
        """
        mass_matrix_jac_S_lambda_s_jac_S = inv(self.mass_matrix) @ self.jac_s.transpose() @ self.lambda_s @ self.jac_s
        size = np.shape(mass_matrix_jac_S_lambda_s_jac_S[0])[0]
        identity_matrix = np.identity(size)
        self.nullspace_s = identity_matrix - mass_matrix_jac_S_lambda_s_jac_S

    def lambda_s_update(self):
        """
        update the inertia matrix s Hutter paper between (4) and (5)
        """
        self.lambda_s = inv(self.jac_s @ inv(self.mass_matrix) @ self.jac_s.transpose())

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

    def mass_matrix_EE_update(self):
        self.mass_matrix_ee = inv(self.jac_s * inv(self.mass_matrix) * self.jac_s.transpose())

    def b_vector_update(self):
        """
        calculates the vector b which acounts for coriolis and centrifugal force
        InverseDynamics: This function computes the generalized forces (actuations of the internal joints (output)) from given generalized states,
            velocities, and accelerations. Computes inverse dynamics with the Newton-Euler Algorithm
        NonlinearEffects: Computes the coriolis forces
        """
        rbdl.NonlinearEffects(self.model, self.state.q, self.state.qd, self.b_vector)

    def spring_force_update(self):
        foot_id = self.model.GetBodyId("foot")
        footInBase = rbdl.CalcBodyToBaseCoordinates(self.model, self.state.q, self.model.GetBodyId("foot"), np.zeros(3),
                                                    True)[:2]
        actualComInFoot = self.pos_com - footInBase
        if self.impact_com is not None:  # TODO. what todo if impactCom was not updated yet with actualcom --> error if impactCom is None.
            impactComInFoot = self.impact_com - footInBase
        else:
            impactComInFoot = actualComInFoot
        directionVector = actualComInFoot / np.linalg.norm(actualComInFoot)
        springLegDelta = np.linalg.norm(impactComInFoot) - np.linalg.norm(actualComInFoot) + self.leg_length
        self.springLegForce = self.spring_stiffness * (springLegDelta) * (directionVector)

    def SpaceControlForce(self):
        self.spaceControlForce = self.lambda_star * (1 / self.robot_mass) * (
                    self.springLegForce + self.robot_mass * self.g) + self.mu_star + self.p_star

    def update(self):
        logging.debug("update center of gravity")
        self.center_of_gravity_update()
        logging.debug("update vel center of gravity")
        self.vel_center_of_gravity_upate
        logging.debug("update current leg length")
        self.current_leg_length_update()
        logging.debug("update current leg angle")
        self.current_leg_angle_update()
        logging.debug("update spring force")
        self.spring_force_update()
        logging.debug("update jacobian s")
        self.jacobian_s_update()
        logging.debug("update mass matrix")
        self.mass_matrix_update()
        logging.debug("update mass matrix ee")
        self.mass_matrix_EE_update
        logging.debug("update lambda s")
        self.lambda_s_update()
        logging.debug("update nullspace s")
        self.nullspace_s_update()
        logging.debug("update jacobian cog")
        self.jacobian_cog_update()
        logging.debug("update jacobian star")
        self.jacobian_star_update()
        logging.debug("update lambda star")
        self.lambda_star_update()
        logging.debug("update b vector")
        self.b_vector_update()
        logging.debug("update p star")
        self.p_star_update()
        logging.debug("update mu star")
        self.mu_star_update()
        logging.debug("update space control force")
        self.SpaceControlForce()
