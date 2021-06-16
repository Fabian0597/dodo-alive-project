import sys
import pathlib

basefolder = str(pathlib.Path(__file__).parent.absolute())
sys.path.append(basefolder + '/../rbdl-orb/build/python/')

import rbdl
import numpy as np
from scipy.integrate import solve_ivp
import math
import os


class WalkerState:
    def __init__(self, model):
        self.q = np.zeros(model.qdot_size)
        self.qd = np.zeros(model.qdot_size)
        self.qdd = np.zeros(model.qdot_size)
        self.tau = np.zeros(model.qdot_size)


class ArticulatedLegWalker:
    def __init__(self, leg_model_path):
        leg_model = rbdl.loadModel(leg_model_path)

        self.dof_count = leg_model.qdot_size

        foot_id = leg_model.GetBodyId("foot")

        self.footIds = np.array([foot_id])

        constraint_left = rbdl.ConstraintSet()
        constraint_right = rbdl.ConstraintSet()

        planeY = np.array([0, 1, 0], dtype=np.double)
        planeX = np.array([1, 0, 0], dtype=np.double)

        constraint_left.AddContactConstraint(leftFootId, np.zeros(3), planeX)
        constraint_left.AddContactConstraint(leftFootId, np.zeros(3), planeY)

        constraint_right.AddContactConstraint(rightFootId, np.zeros(3), planeX)
        constraint_right.AddContactConstraint(rightFootId, np.zeros(3), planeY)

        constraint_left.Bind(model_left)
        constraint_right.Bind(model_right)

        self.models = [model_left, model_right]
        self.constraints = [constraint_left, constraint_right]

        self.discrete_state = 0

        self.csv = open(basefolder + '/animation.csv', 'w')

    def solve(self, steps=30, timestep=0.001, des_position, state):
        """

        :param steps: iterations / steps of the integration loop of the solver
        :param des_position: desired position target for the robot
        :param state: inital state of the robot's leg (q, qd)
        :return:
        """

        # g function for solver (stop integrating when g=0)
        def g(t, y):
            state = State(self.models[self.discrete_state])
            state.q = y[:self.dof_count]
            state.qd = y[self.dof_count:]

            r_foot = rbdl.CalcBodyToBaseCoordinates(self.models[self.discrete_state], state.q,
                                                    self.footIds[self.discrete_state], np.zeros(3), True)
            if (abs(state.q[3]) < 5.0 * math.pi / 180.0):  # Avoid scuffing
                return 100.0
            else:
                return r_foot[1]
            return r_foot[1]

        g.terminal = True

        t = 0
        dt = timestep # TODO use
        # solver (integration loop)
        for i in range(0, steps):
            iteration = i + 1

            # updateCalculation()

            state_q = state[:model.dof_count]
            floatingbase_id = self.models[0].GetBodyId("floatingBase")
            floatingbase_in_base = rbdl.CalcBodyToBaseCoordinates(
                self.models[0], state_q,
                self.footIds[0], np.zeros(3), True)
            foot_in_base = rbdl.CalcBodyToBaseCoordinates(
                self.models[0], state_q,
                self.footIds[0], np.zeros(3), True)



            solver = solve_ivp(self.f, [t, t + 5], state, events=[g])

            for i in range(0, len(solver.t)):
                row = solver.y.T[i]
                self.log(solver.t[i], row[:self.dof_count])
            t = solver.t[-1]

            # toggle state
            self.discrete_state = abs(self.discrete_state - 1)

            # CompositeRigidBodyAlgorithm
            state = solver.y.T[-1]
            new_qd = np.ones(model.dof_count)
            rbdl.ComputeConstraintImpulsesDirect(self.models[self.discrete_state], state[:self.dof_count],
                                                 state[self.dof_count:], self.constraints[self.discrete_state], new_qd)

            state[self.dof_count:] = new_qd

    def f(self, t, y):
        state = State(self.models[self.discrete_state])
        state.q = y[:self.dof_count]
        state.qd = y[self.dof_count:]

        if (self.discrete_state == 0):
            state.tau[3] = 50.0 * (15.0 * math.pi / 180.0 - state.q[3]) - 5.0 * state.qd[3]
        else:
            state.tau[3] = 50.0 * (-15.0 * math.pi / 180.0 - state.q[3]) - 5.0 * state.qd[3]

        rbdl.ForwardDynamicsConstraintsDirect(self.models[self.discrete_state], state.q, state.qd, state.tau,
                                              self.constraints[self.discrete_state], state.qdd)

        res = np.zeros(2 * self.dof_count)
        res[:self.dof_count] = state.qd
        res[self.dof_count:] = state.qdd
        return res

    def log(self, t, q):
        if self.csv is not None:
            self.csv.write(str(t) + ", " + ", ".join(map(str, q)) + "\n")

    def meshup(self):
        if self.csv is not None:
            self.csv.close()
        os.system("meshup " + basefolder + "/compass.lua " + basefolder + "/animation.csv")


def deg_to_rad(deg):
    return deg * math.pi / 180.0

if __name__ == "__main__":
    model = ArticulatedLegWalker(
        leg_model_path= basefolder + "/compass.lua"
    )

    # q : The model's initial position (x_cog, y_cog) and angles between links (J1, J2)
    q = np.array([0.0, 1.0, deg_to_rad(0.0), deg_to_rad(25.0)])

    # qd: The model's initial velocity.
    qd = np.zeros(model.dof_count)
    new_qd = np.ones(model.dof_count)

    rbdl.ComputeConstraintImpulsesDirect(model.models[model.discrete_state], q, qd,
                                         model.constraints[model.discrete_state], new_qd)
    qd = new_qd

    model.solve(state=np.concatenate((q, qd), axis=0))
    model.meshup()

