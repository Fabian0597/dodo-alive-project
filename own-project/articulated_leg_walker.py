import sys
import pathlib

basefolder = str(pathlib.Path(__file__).parent.absolute())
sys.path.append(basefolder + '/../rbdl-orb/build/python/')

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


class ArticulatedLegWalker:
    def __init__(self, leg_model_path):
        #instance of rbdl model is loaded
        #flight model
        self.leg_model = rbdl.loadModel(leg_model_path)
        #stance model
        self.leg_model_constraint = rbdl.loadModel(leg_model_path)

        # get size of the generalized cooridnates   
        self.dof_count = leg_model.qdot_size

        # initialize generalized coordinates, velocities, accelerations and tau 
        self.q = np.zeros(self.dof_count)
        self.qd = np.zeros(self.dof_count)
        self.qdd = np.zeros(self.dof_count)
        self.tau = np.zeros(self.dof_count)

        self. constraint = rbdl.ConstraintSet()
        foot_id = self.leg_model.GetBodyId('foot')
        constraint.AddContactConstraint(foot_id, np.zeros(3), planeX)
        planeY = np.array([0, 1, 0], dtype=np.double)
        planeX = np.array([1, 0, 0], dtype=np.double)
        self.constraint.Bind(self.leg_model_constraint)

        self.csv = open(basefolder + '/animation.csv', 'w')

    def solve(self, t_init, t_final, state):
        """
        integrate to the output of the forward kinematics which is res = [qd, qdd] to get new state y = [q, qd]

        :param steps: number of steps during integration
        :param timestep: stepsize for integration loop of the solver
        :param des_position: desired position target for the robot
        :param state: inital state of the robot's leg (q, qd)
        :return:
        """
    

        # event function for intergrating solver of flight phase. When g(t,y) = 0 solver stops --> this event marks the contact of foot with ground
        def g(t, y):
            state = State(self.leg_model)
            state.q = y[:self.dof_count]
            state.qd = y[self.dof_count:]

            # Returns the base coordinates of the point (0,0,0) (--> origin of body frame) given in body coordinates of the foot
            foot_id = self.leg_model.GetBodyId('foot')
            foot_pos = rbdl.CalcBodyToBaseCoordinates(self.leg_model, state.q, self.leg_model, np.zeros(3), True)
            return foot_pos[1]
        g.terminal = True


        # Solve an initial value problem for a system of ordinary differential equations (ode)
        # Interval of integration (t_init,, t_final).
        # state = initial state
        # function to be integrated = f
        # it solves the ode until it reaches the event g(y,t)=0 so the foot hits the ground and has (y height 0)
        solver = solve_ivp(self.f, [t_init, t_final], state, events=[g])

        # iterate over internal states saved by the solver during integration
        for i in range(0, len(solver.t)):
            # returns the value of the solution y=[q,qd] at time stamp T[i] from the interval of integration
            row = solver.y.T[i]
            # log the time stamp T[i] from the interval of integration and the corresp first half of the y vector solution which contains q and not qd
            self.log(solver.t[i], row[:self.dof_count])

        # set initial time t and state for next iteration which is last element of stored solver values
        t_impact = solver.t[-1]
        state = solver.y.T[-1]

        # Calculation of change of velocity through impact
        # qd_minus is the generalized velocity before and qd_plus the generalized velocity after the impact
        q_minus = state[:self.dof_count] #first half of state vector is q
        qd_minus = state[self.dof_count:] #second half of state vector is qd
        qd_plus = np.ones(model.dof_count)

        rbdl.ComputeConstraintImpulsesDirect(self.leg_model, q_minus, qd_minus, self.constraints[self.discrete_state], qd_plus)

        # replace generalized velocity in state vector for next iteration by calculated velocity after ground impact
        state[self.dof_count:] = qd_plus

        solver = solve_ivp(self.f_stance, [t_impact, t_final], state, events=[g_stance])

        # iterate over internal states saved by the solver during integration
        for i in range(0, len(solver.t)):
            # returns the value of the solution y=[q,qd] at time stamp T[i] from the interval of integration
            row = solver.y.T[i]

            # log the time stamp T[i] from the interval of integration and the corresp first half of the y vector solution which contains q and not qd
            self.log(solver.t[i], row[:self.dof_count])
        

    def f(self, t, y):
        """
        forward kinematics: computes the generalized accelerations from given generalized states, velocities and forces during flight
        params t: time
        params y: generalized cooridnates and their derivative y=[q, qd].transpose()
        return: derivative yd = [qd, qdd]
        """
        #state = State(self.models[self.discrete_state])

        #state contains q (first half) and qd (second half) from the state space y 
        state.q = y[:self.dof_count]
        state.qd = y[self.dof_count:]
        """
        TODO: what is tau?
        if (self.discrete_state == 0):
            state.tau[3] = 50.0 * (15.0 * math.pi / 180.0 - state.q[3]) - 5.0 * state.qd[3]
        else:
            state.tau[3] = 50.0 * (-15.0 * math.pi / 180.0 - state.q[3]) - 5.0 * state.qd[3]
        """
        # calculate qdd from q, qd, tau, fext for the model with the forward dynamics
        #rbdl.ForwardDynamicsConstraintsDirect(self.leg_model, state.q, state.qd, state.tau, self.constraints[self.discrete_state], state.qdd) --> this is just done during stance
        rbdl.ForwardDynamics(self.leg_model, state.q, state.qd, state.tau, state.qdd)

        # return res = [qd, qdd] from the forwarddynamics which can then be given to the solver to be integrated by the ivp_solver
        res = np.zeros(2 * self.dof_count)
        res[:self.dof_count] = state.qd
        res[self.dof_count:] = state.qdd
        return res

    def f_stance(self, t, y):
        """
        forward kinematics: computes the generalized accelerations from given generalized states, velocities and forces
        params t: time
        params y: generalized cooridnates and their derivative y=[q, qd].transpose()
        return: derivative yd = [qd, qdd]
        """
        #state = State(self.models[self.discrete_state])

        #state contains q (first half) and qd (second half) from the state space y 
        state.q = y[:self.dof_count]
        state.qd = y[self.dof_count:]
        
        """
        TODO: what is tau?
        if (self.discrete_state == 0):
            state.tau[3] = 50.0 * (15.0 * math.pi / 180.0 - state.q[3]) - 5.0 * state.qd[3]
        else:
            state.tau[3] = 50.0 * (-15.0 * math.pi / 180.0 - state.q[3]) - 5.0 * state.qd[3]
        """

        # calculate qdd from q, qd, tau, fext for the model with the forward dynamics
        rbdl.ForwardDynamicsConstraintsDirect(self.models[self.discrete_state], state.q, state.qd, state.tau, self.constraint, state.qdd)

        # return res = [qd, qdd] from the forwarddynamics which can then be given to the solver to be integrated by the ivp_solver
        res = np.zeros(2 * self.dof_count)
        res[:self.dof_count] = state.qd
        res[self.dof_count:] = state.qdd
        return res

    def log(self, t, q):
        """
        write a time sequence of generalized coordinates in csv file which is calculated by the ivp_solver
        """
        if self.csv is not None:
            # define the string we want to put into the csv file which is the time t and the value of the generalized cooridnates q for this time stamp
            self.csv.write(str(t) + ", " + ", ".join(map(str, q)) + "\n")

    def meshup(self):
        """
        creates the vision for the leg
        """
        if self.csv is not None:
            self.csv.close()
        #open meshup from python
        os.system("meshup " + basefolder + "/articulatedLeg.lua " + basefolder + "/animation.csv")



def deg_to_rad(deg):
    return deg * math.pi / 180.0

if __name__ == "__main__":
    model = ArticulatedLegWalker(
        leg_model_path= basefolder + "/articulatedLeg.lua"
    )
    
    # q : The model's initial position (x_cog, y_cog) and angles between links (J1, J2)
    q = np.array([0.0, 1.0, deg_to_rad(0.0), deg_to_rad(25.0)])

    # qd: The model's initial velocity.
    qd = np.zeros(model.dof_count)

    state=np.concatenate((q, qd), axis=0)

    t_init = 0
    t_final = 2
    mode.solve(t_init, t_final_state)
    model.meshup()
