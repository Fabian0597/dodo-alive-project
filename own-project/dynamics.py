import math

import numpy as np
import rbdl

from articulated_leg_walker import WalkerState


class DynamicsState: # TODO rename

    def __init__(self, model, walker_state: WalkerState, timestep):
        self.state = WalkerState()
        self.timestep = timestep

        self.model = model
        self.robot_mass = None
        self.jac_cog = None # jacobianCog
        self.jac_d_cog = None # derivative of jacobian
        self.jac_s = None
        self.jac_base_s = None
        self.jac_s_dot = None
        self.jac_star = None
        self.center_of_mass = None #actualCom
        self.leg_length = None
        self.leg_angle = None


    def calc_numerical_gradient(self, x_new, x_old, step_size):
        return (x_new - x_old) /step_size


    def center_of_gravity_update(self):
        """
        Updates the center of gravity/mass
        """
        num_of_joints = len(self.model.mBodies)
        cog_mass_weighted = np.zeros(3, 1);
        mass_sum = 0
        for i in range(num_of_joints):
            cog_in_body = self.model.mBodies[i].mCenterOfMass
            cog_in_world = rbdl.CalcBodyToBaseCoordinates(self.model, self.state.q, i, cog_in_body, True)
            cog_mass_weighted += cog_in_world * self.model.mBodies[i].mMass
            mass_sum = mass_sum + self.model.mBodies[i].mMass

        self.center_of_mass = cog_mass_weighted / mass_sum

    def current_leg_length_update(self):
        """
        Updates the leg length
        """
        foot_id = self.model.GetBodyId("foot")
        pos_foot = rbdl.CalcBodyToBaseCoordinates(self.model, self.state.q, foot_id, np.zeros(3), True)
        self.leg_length = np.linalg.norm(self.center_of_mass - pos_foot, ord=2)

    def current_leg_angle_update(self):
        """
        Updates the leg angle
        """
        foot_id = self.model.GetBodyId("foot")
        pos_foot = rbdl.CalcBodyToBaseCoordinates(self.model, self.state.q, foot_id, np.zeros(3), True)
        dx = pos_foot[0] - self.center_of_mass[0]
        dy = pos_foot[1] - self.center_of_mass[1]
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

        rbdl.CalcPointJacobian(self.model, self.state.q, self.model.GetBodyId("floatingBase"), np.zeros(3), jac_base, True)
        rbdl.CalcPointJacobian(self.model, self.state.q, self.model.GetBodyId("foot"), np.zeros(3), jac_foot, True)
        self.jac_base_s = jac_foot - jac_base
        self.jac_s = jac_foot
        self.jac_s_dot = self.calc_numerical_gradient(self.jac_s, pre_jac_s, self.timestep)


    def jacobian_star_update(self):
        pass
        #actuationMatrix = np.zeros(2,2) np.eye(act)
        #actuationMatrixNullspaceS =

    #def lambda_s_update(self):
    #    self.lambda_s =

    #def com_vel(self):
    #    jac = self.jac_cog()
    #    return jac @ self.qd



    def update(self):
        pass
