#!/usr/bin/python3
import sys
import pathlib
basefolder = str(pathlib.Path(__file__).parent.absolute())
sys.path.append(basefolder+'/../rbdl-orb/build/python/')

import rbdl
import numpy as np
from scipy.integrate import solve_ivp
import math
import os
from enum import Enum
from simple_pid import PID #python3 -m pip install simple-pid

import RobotPlotter

lastCom = None

class State:
    __com = None
    __jac = None
    __model = None

    def __init__(self, model, y = None):
        self.q = np.zeros(model.qdot_size)
        self.qd = np.zeros(model.qdot_size)
        self.qdd = np.zeros(model.qdot_size)
        self.tau = np.zeros(model.qdot_size)

        self.__model = model

        if y is not None:
            self.q = y[:model.dof_count]
            self.qd = y[model.dof_count:]

    #lazy loaded com
    def com(self):
        if self.__com is None:
            mass_com_x = 0
            mass_com_y = 0
            totalMass = 0       
            for i in range(2, len(self.__model.mBodies)):
                com_i = self.__model.GetBody(i).mCenterOfMass
                com_i_base = rbdl.CalcBodyToBaseCoordinates(self.__model, self.q, i, com_i, False)
                mass_i = self.__model.GetBody(i).mMass
                if max(abs(com_i_base[0:2]-self.q[0:2])) > 1:
                    return None#something is wrong
                mass_com_x += com_i_base[0] * mass_i
                mass_com_y += com_i_base[1] * mass_i
                totalMass += mass_i

            com_x = mass_com_x / totalMass
            com_y = mass_com_y / totalMass

            self.__com = np.array([com_x, com_y, 0.])
        return self.__com

    def jac_cog(self):
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

    def com_vel(self):
        jac = self.jac_cog()
    
        return jac @ self.qd


        


class Mode(Enum):
    STANCE = 0
    FLIGHT = 1

class AritculatedLeg:
    # SLIP Model, definitions 
    slip_mass = 50#is equal to totalmass
    slip_stiffness = 10000.
    slip_length_zero = 0.6 #start slip length
    slip_length = 0.6#slip length in relaxed state (is modified in impact compensation)
    
    impactQ = np.array([55. * math.pi/180.0, -100. * math.pi/180.0])
    impactAngle = 78 * math.pi / 180

    vel_des = 3#raibertP
    apex_des = None#apexP

    forceStop = False

    #fligt phase (helper variables)
    flight_target_q = None
    flight_last_height = -100000
    flight_fallen_for_how_many_steps = 0

    com_heights = []#save com heights for visualization

    def __init__(self):
        self.model = rbdl.loadModel(basefolder+"/articulated_leg.lua")

        self.dof_count = self.model.qdot_size

        planeY = np.array([0,1,0], dtype=np.double)
        planeX = np.array([1,0,0], dtype=np.double)

        #stance phase
        self.constraintSetStance = rbdl.ConstraintSet()
        self.constraintSetStance.AddContactConstraint(self.model.GetBodyId("foot"), np.zeros(3), planeX)
        self.constraintSetStance.AddContactConstraint(self.model.GetBodyId("foot"), np.zeros(3), planeY)
        self.constraintSetStance.Bind(self.model)
        #flight phase
        self.emptySet = rbdl.ConstraintSet()
        self.emptySet.Bind(self.model)

        #init phase is stance phase
        self.mode = Mode.FLIGHT
        

        #plot
        def stopByUser():
            self.forceStop = True
        self.robotPlotter = RobotPlotter.RobotPlotter(self.model, stopByUser)
        self.robotPlotter.setAxisLimit((-1,6),(-1,2))
        for el in ["floatingBase","link1","link2", "foot"]:
            self.robotPlotter.addBodyId(self.model.GetBodyId(el))
        #plot gound
        self.robotPlotter.showPoints(0,"k-",[-1000,0], [1000, 0])

        self.csv = open(basefolder+'/animation.csv', 'w')
        self.ffcsv = open(basefolder+'/forces.ff', 'w')
        self.ff = np.zeros(9)

        self.old_t = -0.001
        self.old_J_s = np.array([[1, 1, 1, 1,],[1, 1, 1, 1,]], dtype=float)
        self.old_J_cog = np.array([[1, 1, 1, 1,],[1, 1, 1, 1,]], dtype=float)
        self.J_cog_grad = np.zeros(self.old_J_cog.shape)
        self.J_s_grad = np.zeros(self.old_J_s.shape)

        self.old_com = np.zeros(3)
        self.com_grad = None

        #used for g function
        self.lastSpringForce = -10000
        self.lastFootPos = -10000

        #flight PID
        self.pid = PID(10000, 300, 10000)

    #returns correct constraintSet depending on the current Mode
    def constraintSet(self):
        if self.mode == Mode.STANCE:
            return self.constraintSetStance
        else:
            return self.emptySet


    def solve(self):
        x = np.zeros(2*self.dof_count)
        #init vector
        x[0] = 0.0
        x[1] = 1.8

        #init joints
        x[2] = self.impactQ[0]
        x[3] = self.impactQ[1]

        #init speed
        x[4] = 2.5

        #calc init impulses
        qd_plus = np.ones(self.dof_count)
        rbdl.ComputeConstraintImpulsesDirect(self.model, x[:self.dof_count], x[self.dof_count:], self.constraintSet(), qd_plus)
        x[self.dof_count:] = qd_plus

                
        
        #g function for solver (stop integrating when g=0)
        def g(t, y):
            if self.forceStop:
                return 0#stopped by user
            #update root visualization
            state = State(self.model, y)
            self.robotPlotter.updateRobot(t, state.q)        

            com = state.com()
            if com is None:
                com = self.com_heights[-1]#use last height if calcualtion is failed
            
            #show com path
            if(len(self.com_heights) == 0 or max(abs(self.com_heights[-1] - com)) > 0.1):
                self.com_heights.append(com)
                self.robotPlotter.showPoints(t,'k--',*self.com_heights)

            #visualize slip model
            footpos = rbdl.CalcBodyToBaseCoordinates(self.model, state.q, self.model.GetBodyId("foot"), np.zeros(3), False)    
            self.robotPlotter.showPoints(t, 'go-', com, footpos)
            #visualize com velocity
            self.robotPlotter.showVector(t,com[0:2], state.com_vel(), 'm')


            #emergency stop if fallen trough ground
            if footpos[1] < -2:
                return 0

            if self.mode == Mode.STANCE:
                distance_foot_com = com - footpos
                slip_new_length = np.linalg.norm(distance_foot_com, ord=2)
                f = self.slip_stiffness*(self.slip_length - slip_new_length)

                #only detect if springforce decreases
                if self.lastSpringForce > f:

                    floatingBasePos = rbdl.CalcBodyToBaseCoordinates(self.model, state.q, self.model.GetBodyId("floatingBase"), np.zeros(3), False)  
                    baseFootDist = np.linalg.norm(floatingBasePos-footpos, ord=2)
                    if f < 0.08 or abs(baseFootDist) > 1.55:#f small or fully extended
                        self.lastSpringForce = -10000
                        return 0#STOP
                
                self.lastSpringForce = f
                return 1
            if self.mode == Mode.FLIGHT:

                #only detect if falling down
                if self.lastFootPos > footpos[1] and self.lastFootPos > 0:
                    if footpos[1] <= 0:
                        self.lastFootPos = -10000
                        return 0#STOP
                    else:
                        return footpos[1]
                    
                self.lastFootPos = footpos[1]

                return 1
            return 1
        g.terminal = True #stop integration when g function raises event

        #how much steps should the robot make (-> steps = 2*FLIGHT/STANCE switch)
        steps = 30
        t = 0
        #solver (integration loop)
        for i in range(0, steps*2):
            self.robotPlotter.print(t, "now in Mode: " + str(self.mode))
            solver = solve_ivp(self.f, [t, t+2], x, max_step=0.1, events=[g])# init integrater. integrate max t+2 (2 seconds) without a raise of g event
   
            #event raised or 2 secons expired -> log results form solver
            for i in range(0, len(solver.t)):
                row = solver.y.T[i]
                self.log(solver.t[i], row[:self.dof_count])
                self.fflog(solver.t[i])

            #stopped by user
            if self.forceStop:
                break
            
            #save end-time of solver for next start
            t = solver.t[-1]

            #toggle stateoutt
            self.mode = Mode.FLIGHT if self.mode == Mode.STANCE else Mode.STANCE

            #delete target value for flight phase
            self.flight_target_q = None
            self.flight_last_height = -100000
            self.flight_fallen_for_how_many_steps = 0

            #take robot snapshot
            x = solver.y.T[-1]
            state = State(self.model, x)

            #compute impulses
            if self.mode == Mode.STANCE:
                self.calculate_new_slip_length(state, t)
                vel = np.linalg.norm(state.com_vel(), ord=2)

            rbdl.ComputeConstraintImpulsesDirect(self.model, x[:self.dof_count], x[self.dof_count:], self.constraintSet(), qd_plus)
            x[self.dof_count:] = qd_plus


    def log(self, t, q):
        if self.csv is not None:
            self.csv.write(str(t) + ", " + ", ".join(map(str, q)) + "\n")

    def fflog(self, t):
        if self.ffcsv is not None:
            scale = abs(max(self.ff[3:6].min(), self.ff[3:6].max(), key=abs))
            if scale != 0:
                self.ff[3:6] = (self.ff[3:6] * 200 / scale)#scaled to 200
            self.ffcsv.write(str(t) + ", "+ ", ".join(map(str, self.ff)) + "\n")

    def playbackMode(self):
        self.robotPlotter.playbackMode()

    def meshup(self):
        if self.csv is not None:
            self.csv.close()
        os.system("meshup "+basefolder+"/articulated_leg.lua "+basefolder+"/animation.csv "+basefolder+"/forces.ff")
    
    def controllerStance(self, state, t):
        gravity = self.model.gravity   

        #Js fußpunkt 2ter link -> point ist fußspitze (die in kontakt mit boden ist)
        J_s = np.zeros((3, self.model.q_size))

        footpos = rbdl.CalcBodyToBaseCoordinates(self.model, state.q, self.model.GetBodyId("foot"), np.zeros(3), False)
        rbdl.CalcPointJacobian(self.model, state.q, self.model.GetBodyId("foot"), np.zeros(3), J_s, False)

        J_s = np.delete(J_s, 2, 0)

        #mass matrix
        M = np.zeros((self.model.q_size, self.model.q_size))
        rbdl.CompositeRigidBodyAlgorithm(self.model, state.q, M, False)
        M_inv = np.linalg.inv(M)

        #actuation matrix S -> only last joint is actuated
        S = np.zeros((2, self.model.q_size))
        S[0,2] = 1
        S[1,3] = 1

        #Jcog ist gesamtes (vom roboter)
        com = state.com()

        if com is None:
            com = self.com_heights[-1]

        J_cog = state.jac_cog()

        #b coriolis/centrifugal -> TODO: b is a generalized force -> why do we need to peoject it to the COG?
        b = np.zeros(self.model.q_size)
        rbdl.NonlinearEffects(self.model, state.q, state.qd, b)

        lambda_s = np.linalg.inv(J_s @ M_inv @ J_s.T)

        N_s = np.eye(4) - M_inv @ J_s.T @ lambda_s @ J_s

        J_star = J_cog @ M_inv @ (S @ N_s).T @ np.linalg.inv(S @ N_s @ M_inv @ (S @ N_s).T)# TODO: check why pinv needed -> S was the problem!
        J_star = J_star[0:2]

        
        lambda_star = np.linalg.pinv(J_cog[0:2] @ M_inv @ (S @ N_s).T @ J_star.T)
        # lambda_star = np.linalg.inv(J_s @ M_inv @ S @ N_s @ J_s.T) # problem with dimensions

        # calculate derivative of Jacobians
        if t-self.old_t > 0.01:
            self.J_cog_grad = (1/(t-self.old_t)) * (J_cog[0:2] - self.old_J_cog[0:2])
            self.J_s_grad = (1/(t-self.old_t)) * (J_s - self.old_J_s)


        self.old_t = t
        self.old_J_cog = J_cog
        self.old_J_s = J_s

        term1 = lambda_star @ J_cog[0:2] @ M_inv @ N_s.T @ b 
        term2 = lambda_star @ self.J_cog_grad @ state.qd
        term3 = lambda_star @ J_cog[0:2] @ M_inv @ J_s.T @ lambda_s @ self.J_s_grad @ state.qd 
        coriolis_grav_part = term1 - term2 + term3 

        # compute distance foot and COM
        distance_foot_com = com - footpos
        slip_new_length = np.linalg.norm(distance_foot_com, ord=2)
        angle = np.arctan2(distance_foot_com[1],distance_foot_com[0])
        

        deltaX = self.slip_length - slip_new_length

        if deltaX < 0:
            deltaX = 0#do not set negative forces -> energy loss
            slip_force = np.zeros(2)
        else:
            slip_force = self.slip_stiffness*(np.array([deltaX*math.cos(angle), deltaX*np.sin(angle), 0])) + self.slip_mass*gravity
            slip_force = slip_force[0:2]

        # Control law 
        torque_new = J_star.T @ (lambda_star @ ((1/self.slip_mass)*slip_force))
        coriolis = J_star.T @ coriolis_grav_part
        
        state.tau[2:4] = coriolis + torque_new

        #show force-log in mehup and matplotlib
        self.robotPlotter.showVector(t,com[0:2],slip_force, 'r')
        self.ff = np.zeros(9)
        self.ff[0:3] = com
        self.ff[3:5] = slip_force
        self.fflog(t)
        
        return state


    def calculate_new_slip_length(self, state, t):
        J_cog = state.jac_cog()
        J_s = np.zeros((3, self.model.q_size))
        rbdl.CalcPointJacobian(self.model, state.q, self.model.GetBodyId("foot"), np.zeros(3), J_s, False)
        J_s = np.delete(J_s, 2, 0)
        #mass matrix
        M = np.zeros((self.model.q_size, self.model.q_size))
        rbdl.CompositeRigidBodyAlgorithm(self.model, state.q, M, False)
        M_inv = np.linalg.inv(M)
        lambda_s = np.linalg.inv(J_s @ M_inv @ J_s.T)
        N_s = np.eye(4) - M_inv @ J_s.T @ lambda_s @ J_s
        matrix = J_cog.T @ J_cog - N_s.T @ J_cog.T @ J_cog @ N_s

        post_vel = np.linalg.norm(J_cog @ N_s @ np.linalg.pinv(J_cog) @ state.com_vel(),ord=2)
        self.robotPlotter.print(t, f"calculated post vel: {post_vel}")
        new_length = (1/self.slip_stiffness) * self.slip_mass * (state.qd.T @ matrix @ state.qd)
        self.robotPlotter.print(t, "delta length: " + str(new_length))
        new_length = math.sqrt(abs(new_length))
        self.robotPlotter.print(t, "velocity before impulse: " + str(state.qd))
        self.robotPlotter.print(t, "old length: " + str(self.slip_length))
        self.slip_length = self.slip_length_zero +  new_length
        #if self.slip_length > 1.1:
            #print("max reached")
            # self.slip_length = 1.1
        self.robotPlotter.print(t, "new length: " + str(self.slip_length))
        energy = 0.5 * self.slip_mass * state.qd.T @ matrix @ state.qd
        self.robotPlotter.print(t, "Delta Energy: " + str(energy))
        #return self.slip_length_zero - new_length - 0.2


    def controllerFlight(self, state, t):   
        com = state.com() 
        if com is None:
            com = self.com_heights[-1]

        if self.flight_last_height > com[1]:
            self.flight_fallen_for_how_many_steps += 1
        else:
            self.flight_fallen_for_how_many_steps = 0

        #PHASE 1 (started -> keep let position until foot is high enough and could touch ground if moving)
        foot = rbdl.CalcBodyToBaseCoordinates(self.model, state.q, self.model.GetBodyId("foot"), np.zeros(3), False)
        if self.flight_target_q is None and foot[1] < 0.3:
            self.pid.setpoint=state.q[2:4] 

        #PHASE 2 (move leg to initial position (angle calculation for vom is otherwise))
        if self.flight_target_q is None and foot[1] > 0.3:
            self.pid.setpoint=self.impactQ[0:2]

        #PHASE 3 (is apex?)
        if self.flight_last_height > com[1] and self.flight_fallen_for_how_many_steps > 3:
            
            com_vel_rbdl = state.com_vel()

            if self.flight_target_q is None:

                #fixed Angle
                if False:
                    angle_des = self.impactAngle
                    new_point = np.array([com[0]+math.cos(angle_des)*self.slip_length_zero, com[1]-math.sin(angle_des)*self.slip_length_zero,0])
                #raibertP
                if False:
                    x_f = - 0.3 * (com_vel_rbdl[0] - self.vel_des)
                    if self.slip_length_zero**2 - x_f**2 > 0:
                        y_f = -math.sqrt(self.slip_length_zero**2 - x_f**2)
                    else:
                        y_f = 0
                    new_point = np.array([com[0] + x_f, com[1] + y_f])
                #apexP (custom controller)
                if True:
                    if self.apex_des is None:
                        self.apex_des = com[1]
                    angle_des = self.impactAngle - 0.45 * (self.apex_des - com[1])
                    new_point = np.array([com[0]+math.cos(angle_des)*self.slip_length_zero, com[1]-math.sin(angle_des)*self.slip_length_zero,0])

                    self.apex_des = com[1]

                
                
                #build vars for inverse kinematics
                basePos = np.array([state.q[0], state.q[1], 0])
                
                target_coord = np.array([basePos, np.array([new_point[0],new_point[1],0])])

                target_q = np.zeros(self.model.q_size)
                rbdl.InverseKinematics(self.model, state.q, np.array([self.model.GetBodyId('floatingBase'), self.model.GetBodyId('foot')]), np.zeros([2,3]), target_coord, target_q)
                self.flight_target_q = target_q

                #outer PID
                self.pid.setpoint=self.flight_target_q[2:4]

        tau = self.pid(state.q[2:4])
        state.tau[2:4] = tau
        
        self.flight_last_height = com[1]

        return state

    def f(self, t, y):
        #init state
        state = State(self.model, y)
        
        #controller state.tau[3] = ?
        if(self.mode == Mode.STANCE):
            state = self.controllerStance(state, t)
        else:
            state = self.controllerFlight(state, t)

        #compute forwarddynamics (compute yd=[qd, qdd] given y=[q, qd])
        rbdl.ForwardDynamicsConstraintsDirect(self.model, state.q, state.qd, state.tau, self.constraintSet(), state.qdd)

        #result gradient of y as res
        res = np.zeros(2*self.dof_count)
        res[:self.dof_count] = state.qd
        res[self.dof_count:] = state.qdd
        return res

    
if __name__ == "__main__":
    model = AritculatedLeg()
    model.solve()
    model.playbackMode()#keep open matlablib