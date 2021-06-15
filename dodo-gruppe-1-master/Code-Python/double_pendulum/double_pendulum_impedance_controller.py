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
        self.q1_initial = 10*math.pi/180.0
        self.q2_initial = -45*math.pi/180.0
        self.q = np.array([self.q1_initial, self.q2_initial])

        # initialization contact costraints for controller
        # planeY = np.array([0, 1, 0], dtype=np.double)
        # planeZ = np.array([0,0,1], dtype=np.double)

        # self.constraintSet = rbdl.ConstraintSet()

        # self.constraintSet.AddContactConstraint(self.model.GetBodyId("EE"), np.zeros(3), planeY)
        # self.constraintSet.AddContactConstraint(self.model.GetBodyId("EE"), np.zeros(3), planeZ)
        # self.constraintSet.Bind(self.model)

        imagPoint = rbdl.CalcBodyToBaseCoordinates(self.model, self.q, self.model.GetBodyId("imaginaryPoint"), np.zeros(self.model.q_size), False)
        print("x", imagPoint[0])
        print("y", imagPoint[1])

        self.ff = np.zeros(9)

        self.x_desired = np.array([imagPoint[0], imagPoint[1]])

        self.csv = open(basefolder+'/animation.csv', 'w')
        self.ffcsv = open(basefolder+'/forces.ff', 'w')


    
 
    def solve(self):
        init = np.concatenate((self.q, np.zeros(2)))
        solver = ode(self.f)
        solver.set_initial_value(init, 0)
        dt = 0.1
        while solver.t < 10:
            self.log(solver.t)
            self.fflog(solver.t)
            solver.integrate(solver.t + dt)
          

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
            self.csv.write(str(t) + ", " + ", ".join(map(str, self.q)) + "\n")
    def fflog(self, t):
        if self.ffcsv is not None:
            scale = abs(max(self.ff[3:6].min(), self.ff[3:6].max(), key=abs))
            if scale != 0:
                self.ff[3:6] = (self.ff[3:6] * 200 / scale)#scaled to 200
            self.ffcsv.write(str(t) + ", "+ ", ".join(map(str, self.ff)) + "\n")
    def meshup(self):
        if self.csv is not None:
            self.csv.close()
        if self.ffcsv is not None:
            self.ffcsv.close()
        os.system("meshup "+basefolder+"/double_pendulum.lua "+basefolder+"/animation.csv "+basefolder+"/forces.ff")
    def f(self, t, y):
    
        self.q = y[:self.model.q_size]
        self.qdot = y[self.model.q_size:]

        self.tau = np.zeros(self.model.q_size)

        # control if pendulum is in state of maximal displacement (left side) 
        # if self.q[0] <= -1 * self.q1_initial:
            # print("CONTROL")
        
        self.control(t)

        # # apply force
        # if(t > 0):
        #     Q = np.zeros((3, self.model.q_size))
        #     rbdl.CalcPointJacobian(self.model, self.q, self.model.GetBodyId("link2"), np.zeros([0]), Q)
        #     F = np.array([10,0,0])
        #     self.tau = Q.T @ F

  

        # damping 
        # self.tau[0] -= 2*self.qdot[0]
        # self.tau[1] -= 2*self.qdot[1]
        
        #compute forwarddynamics (compute yd=[qd, qdd] given y=[q, qd])

        rbdl.ForwardDynamics(self.model, self.q, self.qdot, self.tau, self.qddot)
        # rbdl.ForwardDynamicsConstraintsDirect(self.model, self.q, self.qdot, self.tau, self.constraintSet, self.qddot)
        return [self.qdot[0], self.qdot[1], self.qddot[0], self.qddot[1]]
        
    def control(self, t):
        # Stiffness matrix
        matrixK_desired = np.array([[1.,0.], [0.,1.]])
        # Damping matrix
        matrixD_desired = np.array([[0.5,0.], [0.,0.5]])
        
        gravity = self.model.gravity[0:2]

        # desired velocity
        v_desired = np.array([0, 0])
        # desired acceleration
        a_desired = np.array([0, 0])
        # desired inertia -> compute it ???
        desiredInertia = np.array([[1.,0.], [0.,1.]])
        desiredInertia_inverted = np.linalg.inv(desiredInertia)

        # Mass matrix
        matrix_M = np.zeros((self.model.q_size, self.model.q_size))
        rbdl.CompositeRigidBodyAlgorithm(self.model, self.q, matrix_M, False)
        matrix_M_inv = np.linalg.inv(matrix_M)

        # Jacobian and transposed Jacobian (2x2 dimesnion)
        jacobian_EE =  np.zeros((3, self.model.q_size)) # must be of dimension 3xdof(q)
        coordinates_EE3d =  rbdl.CalcBodyToBaseCoordinates(self.model, self.q, self.model.GetBodyId("EE"), np.zeros(self.model.q_size), False)
        coordinates_EE = coordinates_EE3d[0:2] # we are only interested in x and y 
        rbdl.CalcPointJacobian(self.model, self.q, self.model.GetBodyId("EE"), np.zeros(2), jacobian_EE)
        jacobian_EE = jacobian_EE[0:2, :] # ignore rotational parts and z part (always zero)
        
        # Inertia = Inverse of Jacobian*MassMatrix*Jacobian_transposed
        inertia = np.linalg.pinv(np.dot(np.dot(jacobian_EE, matrix_M), jacobian_EE.T))
        
        # dynamics task space
        x = coordinates_EE#np.dot(jacobian_EE, self.q)
        x_dot = rbdl.CalcPointVelocity(self.model, self.q, self.qdot, self.model.GetBodyId("EE"), coordinates_EE3d, False)
        x_dot = x_dot[0:2]

        x_dotdot = rbdl.CalcPointAcceleration(self.model, self.q, self.qdot, self.qddot, self.model.GetBodyId("EE"), coordinates_EE3d, False)
        x_dotdot = x_dotdot[0:2]

        #helper vars
        inertiaDotDesiredInertia = np.dot(inertia, desiredInertia_inverted)

        # external Forces -> ???
        externalForce = np.zeros(2)
        

        # Coriolis and Centrifugal terms 
        coriolis = np.zeros(self.model.q_size)
        rbdl.NonlinearEffects(self.model, self.q, self.qdot, coriolis)


        # Resulting control law for joint space
        springForce = np.dot(matrixK_desired, self.x_desired-x)
        damperForce = np.dot(matrixD_desired, v_desired-x_dot)
        jointControlLaw = springForce + damperForce 

        # compute terms step by step
        coriolisTerm = np.dot(jacobian_EE.T, np.dot(inertia, a_desired) + np.dot(coriolis, v_desired))
        springDamperSystem = np.dot(jacobian_EE.T, np.dot(inertiaDotDesiredInertia, jointControlLaw))
        forceTerm = np.dot(jacobian_EE.T, np.dot((inertiaDotDesiredInertia-np.identity(2)), externalForce))
        
        #translated from "Final Report Whole-Body Balance Control" Katharina Hermman, Sadek Cheaib Fig.9
        J_cog = np.zeros((3, self.model.q_size))
        totalMass = 0
        for i in range(len(self.model.mBodies)):#todo: 3 bodies. variable?
            #print("Name",self.model.GetBodyName(i))
            com_i = self.model.GetBody(i).mCenterOfMass
            jac_i = np.zeros((3, self.model.q_size))
            rbdl.CalcPointJacobian(self.model, self.q, i, com_i, jac_i)#wrong jacobian?
            J_cog += self.model.GetBody(i).mMass * jac_i
            totalMass += self.model.GetBody(i).mMass

        J_cog /= totalMass
        J_cog = J_cog[0:2]
        #gravity term g(q) according to Hutter et al (iv)
        gravity_q = J_cog @ matrix_M_inv @ gravity
        #print(jacobian_EE.T @ gravity)
        # j.T @ gravity

        #tested: springDamperSystem (workes); gravity_q (does not work);
        torque_new = -coriolis - springDamperSystem
        #torque_new = gravity_q + coriolisTerm - springDamperSystem + forceTerm

        torque_new = -torque_new

        #visualize force
        self.ff = np.zeros(9)
        self.ff[0:2] = coordinates_EE
        self.ff[3:5] = coriolis
       
        torque_new = np.clip(torque_new, -500, 500)
        if (torque_new.max() > 500) or (torque_new.min() < -500):
            print("clipped", torque_new)

        # apply the resulting torque to the robot
        self.tau = torque_new


        

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

