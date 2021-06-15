import sys
import pathlib
basefolder = str(pathlib.Path(__file__).parent.absolute())
sys.path.append(basefolder+'/../rbdl-orb/build/python/')

import rbdl
import numpy as np
from scipy.integrate import ode
from scipy.integrate import solve_ivp
import math
import os

class DoublePendulum:
    def __init__(self):
        self.model = rbdl.loadModel(basefolder + "/double_pendulum.lua")

        self.q = np.zeros (self.model.q_size)
        self.qdot = np.zeros (self.model.qdot_size)
        self.qddot = np.zeros (self.model.qdot_size)
        self.tau = np.zeros (self.model.qdot_size)

        #init q
        self.q = np.array([20*math.pi/180.0, 20*math.pi/180.0])

        self.csv = open(basefolder+'/animation.csv', 'w')
        self.desired = np.loadtxt(open(basefolder+'/desired.csv', "rb"), delimiter=",")
        self.step = 0
    
 
    def solve(self):
        init = np.concatenate((self.q, np.zeros(2)))
        solver = ode(self.f)
        solver.set_initial_value(init, 0)
        dt = 0.1
        while solver.successful() and solver.t < 20:
            self.log(solver.t)
            solver.integrate(solver.t + dt)
            self.step += 1
          

    def solveWithEvent(self, event):
        init = np.concatenate((self.q, np.zeros(2)))
        solver = solve_ivp(self.f, [0, 10], init, events=[event])

        for i in range(0, len(solver.t)):
            row = solver.y.T[i]
            self.q = row[:self.model.q_size]
            self.qdot = row[self.model.q_size:]
            self.log(solver.t[i])

    def log(self, t):
        if self.csv is not None:
            self.csv.write(str(t) + ", " + ", ".join(map(str, self.q)) + ", " + ", ".join(map(str, self.qdot)) + "\n")

    def meshup(self):
        if self.csv is not None:
            self.csv.close()
        os.system("meshup "+basefolder+"/double_pendulum.lua "+basefolder+"/animation.csv")
    def f(self, t, y):

        self.q = y[:self.model.q_size]
        self.qdot = y[self.model.q_size:]

        self.tau = np.zeros(self.model.q_size)
 
        self.controll()
        # # apply force
        # if(t > 0):
        #     Q = np.zeros((3, self.model.q_size))
        #     rbdl.CalcPointJacobian(self.model, self.q, self.model.GetBodyId("link2"), np.zeros([0]), Q)
        #     F = np.array([10,0,0])
        #     self.tau = Q.T @ F

  

        # damping 
        self.tau[0] -= 2*self.qdot[0]
        self.tau[1] -= 2*self.qdot[1]
        
        rbdl.ForwardDynamics(self.model, self.q, self.qdot, self.tau, self.qddot)
        return [self.qdot[0], self.qdot[1], self.qddot[0], self.qddot[1]]
        
    def controll(self):
        matrixK_d = np.array([[80.,0.], [0.,80.]])
        matrixD_d = np.array([[5.,0.], [0.,5.]])
        q_desired = self.desired[self.step, 1:3]
        q_desired = np.reshape(q_desired, 2)
        q_dot_desired = self.desired[self.step, 3:5]
        q_dot_desired = np.reshape(q_dot_desired, 2)
        tau = np.dot(matrixK_d, np.transpose(q_desired-self.q))+np.dot(matrixD_d, np.transpose(q_dot_desired-self.qdot))

        self.tau[0] = tau[0]
        self.tau[1] = tau[1]
        
        rbdl.ForwardDynamics(self.model, self.q, self.qdot, self.tau, self.qddot)       
        

def start():
    model = DoublePendulum()
    model.solve()
    model.meshup()

def startWithEvent(stopOnEvent):
    model = DoublePendulum()
    model.solve()
    # def event(t, y):
    #     return y[0] > 0
    # event.terminal = stopOnEvent
    
    # model.solveWithEvent(event)
   
    model.meshup()

if __name__ == "__main__":
    startWithEvent(0)

