import sys
import pathlib
basefolder = str(pathlib.Path(__file__).parent.absolute())
sys.path.append(basefolder+'/../rbdl-orb/build/python/')

import rbdl
import numpy as np
from scipy.integrate import solve_ivp
import math
import os

class State:
    def __init__(self, model):
        self.q = np.zeros(model.qdot_size)
        self.qd = np.zeros(model.qdot_size)
        self.qdd = np.zeros(model.qdot_size)
        self.tau = np.zeros(model.qdot_size)

class CompassWalker:
    def __init__(self):
        model_left = rbdl.loadModel(basefolder+"/compass.lua")
        model_right = rbdl.loadModel(basefolder+"/compass.lua")

        self.dof_count = model_right.qdot_size

        leftFootId = model_left.GetBodyId("foot1")
        rightFootId = model_right.GetBodyId("foot2")

        self.footIds = np.array([rightFootId, leftFootId])

        constraint_left = rbdl.ConstraintSet()
        constraint_right = rbdl.ConstraintSet()

        planeY = np.array([0,1,0], dtype=np.double)
        planeX = np.array([1,0,0], dtype=np.double)

        constraint_left.AddContactConstraint(leftFootId, np.zeros(3), planeX)
        constraint_left.AddContactConstraint(leftFootId, np.zeros(3), planeY)

        constraint_right.AddContactConstraint(rightFootId, np.zeros(3), planeX)
        constraint_right.AddContactConstraint(rightFootId, np.zeros(3), planeY)

        constraint_left.Bind(model_left)
        constraint_right.Bind(model_right)

        self.models = [model_left, model_right]
        self.constraints = [constraint_left, constraint_right]

        self.discrete_state = 0

        self.csv = open(basefolder+'/animation.csv', 'w')

    def solve(self):
        x = np.zeros(2*self.dof_count)

        x[0] = 0.0
        x[1] = 1.0

        x[2] = 0.0 * math.pi/180.0
        x[3] = 25.0 * math.pi/180.0

        x[4] = .0

        qd_plus = np.ones(self.dof_count)

        rbdl.ComputeConstraintImpulsesDirect(self.models[self.discrete_state], x[:self.dof_count], x[self.dof_count:], self.constraints[self.discrete_state], qd_plus)
        x[self.dof_count:] = qd_plus
        
        #g function for solver (stop integrating when g=0)
        def g(t, y):
            state = State(self.models[self.discrete_state])
            state.q = y[:self.dof_count]
            state.qd = y[self.dof_count:]

            r_foot = rbdl.CalcBodyToBaseCoordinates(self.models[self.discrete_state], state.q, self.footIds[self.discrete_state], np.zeros(3), True)
            if(abs(state.q[3]) < 5.0 * math.pi/180.0): #Avoid scuffing
                return 100.0
            else:
                return r_foot[1]
            return r_foot[1]
        g.terminal = True

        steps = 30
        t = 0
        #solver (integration loop)
        for i in range(0, steps):
            
            solver = solve_ivp(self.f, [t, t+5], x, events=[g])

            for i in range(0, len(solver.t)):
                row = solver.y.T[i]
                self.log(solver.t[i], row[:self.dof_count])
            t = solver.t[-1]

            #toggle state
            self.discrete_state = abs(self.discrete_state-1)

            #CompositeRigidBodyAlgorithm
            x = solver.y.T[-1]
            rbdl.ComputeConstraintImpulsesDirect(self.models[self.discrete_state], x[:self.dof_count], x[self.dof_count:], self.constraints[self.discrete_state], qd_plus)

            x[self.dof_count:] = qd_plus

    def log(self, t, q):
        if self.csv is not None:
            self.csv.write(str(t) + ", " + ", ".join(map(str, q)) + "\n")

    def meshup(self):
        if self.csv is not None:
            self.csv.close()
        os.system("meshup "+basefolder+"/compass.lua "+basefolder+"/animation.csv")

    def f(self, t, y):
        state = State(self.models[self.discrete_state])
        state.q = y[:self.dof_count]
        state.qd = y[self.dof_count:]

        if(self.discrete_state == 0):
            state.tau[3] = 50.0 * (15.0*math.pi/180.0 - state.q[3]) - 5.0 * state.qd[3]
        else:
            state.tau[3] = 50.0 * (-15.0*math.pi/180.0 - state.q[3]) - 5.0 * state.qd[3]

        rbdl.ForwardDynamicsConstraintsDirect(self.models[self.discrete_state], state.q, state.qd, state.tau, self.constraints[self.discrete_state], state.qdd)

        res = np.zeros(2*self.dof_count)
        res[:self.dof_count] = state.qd
        res[self.dof_count:] = state.qdd
        return res

    
if __name__ == "__main__":
    model = CompassWalker()
    model.solve()
    model.meshup()
