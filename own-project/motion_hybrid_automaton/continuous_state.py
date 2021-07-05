import numpy as np
import rbdl


class ContinuousState:
    __robot_mass = None
    __model = None
    """
    class which helps to handle the robot state more easy.
    It offers basic calculations for the robot configuration which are used several times:
        - position of center of mass
        - velocity of center of mass 
        - robot mass
        - mass matrix
        - inverse of mass matrix
        - Jacobian of com
        - Support Jacobian of foot
        - Support Jacobian of floating base
        - Transformation between q, qd attributes of this class and concatenated vectro x = [q, qd]
        
        
    """
    def __init__(self, model, y=None):
        # generalized coordintaes of robot
        self.q = np.zeros(model.qdot_size)

        #generalized velocities of robot
        self.qd = np.zeros(model.qdot_size)

        if y is not None:
            self.q = y[:model.dof_count]
            self.qd = y[model.dof_count:]

        # Lua model of leg
        self.__model = model

        self._pos_com = None
        self._vel_com = None
        self._mass_matrix = None
        self._inv_mass_matrix = None
        self._jac = None
        self._jac_s = None
        self._jac_base_s = None

    def pos_com(self):
        """
        calculate position of COM for the the given robot configuration
        :return: position of com
        """
        if self._pos_com is None:

            mass_com_x = 0
            mass_com_y = 0
            total_mass = 0

            # go over all links and build average over the links com positions weighted with the mass of each link
            for i in range(2, len(self.__model.mBodies)):

                # get com position in Base coordinates
                com_i = self.__model.GetBody(i).mCenterOfMass
                com_i_base = rbdl.CalcBodyToBaseCoordinates(self.__model, self.q, i, com_i, False)

                # get mass of each link
                mass_i = self.__model.GetBody(i).mMass

                #weight com position with mass
                mass_com_x += com_i_base[0] * mass_i
                mass_com_y += com_i_base[1] * mass_i

                #calculate total mass
                total_mass += mass_i

            #normalize with total mass
            com_x = mass_com_x / total_mass
            com_y = mass_com_y / total_mass

            self._pos_com = np.array([com_x, com_y, 0.])

        return self._pos_com

    def vel_com(self):
        """
        calculate velocity of COM frame for the the given robot configuration
        """
        if self._vel_com is None:
            self._vel_com = self.jac_com() @ self.qd
        return self._vel_com

    def robot_mass(self):
        """
        calculate robot mass for the the given robot configuration
        :return: robot mass
        """
        if self.__robot_mass is None:
            total_mass = 0

            # go over all links and sum up their mass
            for i in range(2, len(self.__model.mBodies)):
                mass_i = self.__model.GetBody(i).mMass
                total_mass += mass_i
            self.__robot_mass = total_mass

        return self.__robot_mass

    def mass_matrix(self):
        """
        calculate mass matrix for the the given robot configuration
        :return: mass matrix
        """
        if self._mass_matrix is None:
            mass_matrix = np.zeros((self.__model.q_size, self.__model.q_size))
            rbdl.CompositeRigidBodyAlgorithm(self.__model, self.q, mass_matrix, False)
            self._mass_matrix = mass_matrix

        return self._mass_matrix

    def inv_mass_matrix(self):
        """
        calculate inverse of mass matrix for the the given robot configuration
        :return: inverse of mass matrix
        """
        if self._inv_mass_matrix is None:
            self._inv_mass_matrix = np.linalg.inv(self.mass_matrix())

        return self._inv_mass_matrix

    def jac_com(self):
        """
        calculate Jacobian in COM frame for the the given robot configuration
        :return: Jacobian in COM frame
        """
        if self._jac is None:
            jac_com = np.zeros((3, self.__model.q_size))
            total_mass = 0

            # go over all links and build average over the links Jacobians weighted with the mass of each link
            for i in range(2, len(self.__model.mBodies)):

                # get com for link
                com_i = self.__model.GetBody(i).mCenterOfMass

                # calculate Jacobian for the com of the link
                jac_i = np.zeros((3, self.__model.q_size))
                rbdl.CalcPointJacobian(self.__model, self.q, i, com_i, jac_i, False)

                # weight Jacobian of com for link with link mass
                jac_com += self.__model.GetBody(i).mMass * jac_i

                #calculate total mass of robot
                total_mass += self.__model.GetBody(i).mMass

            #normalize weighted average of Jacobian of com with total mass
            jac_com /= total_mass
            self._jac = jac_com
        return self._jac

    def jac_s(self):
        """
        calculate support Jacobian for the foot for the the given robot configuration
        :return: support Jacobian for the foot
        """
        if self._jac_s is None:
            jac_s = np.zeros((3, self.__model.q_size))
            rbdl.CalcPointJacobian(self.__model, self.q, self.__model.GetBodyId("foot"), np.zeros(3), jac_s, False)
            self._jac_s = jac_s[:2, :self.__model.dof_count]

        return self._jac_s

    def jac_base_s(self):
        """
        calculate support Jacobian for the floating base for the the given robot configuration
        :return: support Jacobian for the floating base
        """
        if self._jac_base_s is None:
            jac_base = np.zeros((3, self.__model.q_size))
            rbdl.CalcPointJacobian(self.__model, self.q, self.__model.GetBodyId("floatingBase"),
                                   np.zeros(3), jac_base, False)
            self._jac_base_s = self.jac_s() - jac_base[:2, :self.__model.dof_count]

        return self._jac_base_s

    def to_array(self):
        """
        return the q and qd values stored as attributes in the ContinuousState object as as concatenated vector x
        """
        return np.concatenate(self.q, self.qd)
