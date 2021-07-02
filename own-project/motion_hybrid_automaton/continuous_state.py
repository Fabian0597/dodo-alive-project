import numpy as np
import rbdl


class ContinuousState:
    __com = None
    __robot_mass = None
    __jac = None
    __model = None

    def __init__(self, model, y=None):
        self.q = np.zeros(model.qdot_size)
        self.qd = np.zeros(model.qdot_size)

        self.__model = model

        if y is not None:
            self.q = y[:model.dof_count]
            self.qd = y[model.dof_count:]

    def pos_com(self):
        """
        calculate COM for the the given robot configuration
        """
        if self.__com is None:

            mass_com_x = 0
            mass_com_y = 0
            total_mass = 0
            for i in range(2, len(self.__model.mBodies)):
                com_i = self.__model.GetBody(i).mCenterOfMass
                com_i_base = rbdl.CalcBodyToBaseCoordinates(self.__model, self.q, i, com_i, False)
                mass_i = self.__model.GetBody(i).mMass
                # if max(abs(com_i_base[0:2] - self.q[0:2])) > 1:
                #    return None  # something is wrong
                mass_com_x += com_i_base[0] * mass_i
                mass_com_y += com_i_base[1] * mass_i
                total_mass += mass_i

            com_x = mass_com_x / total_mass
            com_y = mass_com_y / total_mass

            self.__com = np.array([com_x, com_y, 0.])

        return self.__com

    def robot_mass(self):

        if self.__robot_mass is None:
            totalMass = 0
            for i in range(2, len(self.__model.mBodies)):
                mass_i = self.__model.GetBody(i).mMass
                totalMass += mass_i
            self.__robot_mass = totalMass

        return self.__robot_mass

    def jac_com(self):
        """
        calculate Jacobian in COM frame for the the given robot configuration
        """
        if self.__jac is None:
            J_cog = np.zeros((3, self.__model.q_size))
            totalMass = 0
            for i in range(2, len(self.__model.mBodies)):
                com_i = self.__model.GetBody(i).mCenterOfMass
                totalMass += self.__model.GetBody(i).mMass
                jac_i = np.zeros((3, self.__model.q_size))
                rbdl.CalcPointJacobian(self.__model, self.q, i, com_i, jac_i, False)
                J_cog += self.__model.GetBody(i).mMass * jac_i
            J_cog /= totalMass
            self.__jac = J_cog
        return self.__jac

    def vel_com(self):
        """
        calculate velocity of COM frame for the the given robot configuration
        """
        jac = self.jac_com()
        return jac @ self.qd

    def to_array(self):
        """
        return the q and qd values stored as attributes in the ContinuousState object as as concatenated vector x
        """
        return np.concatenate(self.q, self.qd)