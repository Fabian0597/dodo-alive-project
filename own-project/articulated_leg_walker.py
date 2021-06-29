
import sys
import pathlib
basefolder = str(pathlib.Path(__file__).parent.absolute())
sys.path.append(basefolder+'/../../rbdl-tum/build/python/')
import rbdl

import numpy as np
from scipy.integrate import solve_ivp
import math
import os
import RobotPlotter

from motion_state_machine import MotionHybridAutomaton
import logging
import sys

root = logging.getLogger()
root.setLevel(logging.DEBUG)

handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.ERROR)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(module)s - %(message)s')
handler.setFormatter(formatter)
root.addHandler(handler)


class ArticulatedLegWalker:
    def __init__(self, leg_model_path, des_com_pos):
        # instance of rbdl model is loaded
        # flight model

        logging.debug("Loading rbdl leg model")
        self.leg_model = rbdl.loadModel(leg_model_path)

        # get size of the generalized coordinates
        self.dof_count = self.leg_model.qdot_size

        logging.debug("set constraints")
        x_plane = np.array([1, 0, 0], dtype=np.double)
        y_plane = np.array([0, 1, 0], dtype=np.double)
        # stance phase
        self.constraintSetStance = rbdl.ConstraintSet()
        self.constraintSetStance.AddContactConstraint(self.leg_model.GetBodyId("foot"), np.zeros(3), x_plane)
        self.constraintSetStance.AddContactConstraint(self.leg_model.GetBodyId("foot"), np.zeros(3), y_plane)
        self.constraintSetStance.Bind(self.leg_model)
        # flight phase
        self.emptySet = rbdl.ConstraintSet()
        self.emptySet.Bind(self.leg_model)

        logging.debug("open csv file")
        def stopByUser():
            self.forceStop = True
        self.robotPlotter = RobotPlotter.RobotPlotter(self.leg_model, stopByUser)
        self.robotPlotter.setAxisLimit((-1, 6),(-1, 4))
        for el in ["floatingBase","link1","link2", "foot"]:
            self.robotPlotter.addBodyId(self.leg_model.GetBodyId(el))
        #plot gound
        self.robotPlotter.showPoints(0,"k-",[-1000,0], [1000, 0])

        self.csv = open(basefolder + '/animation.csv', 'w')
        self.ffcsv = open(basefolder + '/forces.ff', 'w')

        logging.debug("init Motion State Machine")
        self.state_machine = MotionHybridAutomaton(self.leg_model, des_com_pos, self.emptySet, self.constraintSetStance, self.robotPlotter)


    def solve(self, t_init, t_final, init_state):
        """
        integrate to the output of the forward kinematics which is res = [qd, qdd] to get new state y = [q, qd]

        :param steps: number of steps during integration
        :param timestep: step size for integration loop of the solver
        :param des_position: desired position target for the robot
        :param init_state: initial state of the robot's leg (q, qd)
        """
        state = init_state

        while t_init < t_final:
            # Solve an initial value problem for a system of ordinary differential equations (ode)
            # Interval of integration (t_init,, t_final).
            # state = initial state
            # function to be integrated = f
            # it solves the ode until it reaches the event touchdown_event(y,t)=0
            # so the foot hits the ground and has (y height 0)
            solver = solve_ivp(
                fun=self.state_machine.active_state.flow_function,
                t_span=[t_init, t_final],
                y0=state,
                events=self.state_machine.active_state.events
            )

            # iterate over internal states saved by the solver during integration
            for i in range(0, len(solver.t)):
                # returns the value of the solution y=[q,qd] at time stamp T[i] from the interval of integration
                row = solver.y.T[i]
                # log the time stamp T[i] from the interval of integration
                # and the corresp first half of the y vector solution which contains q and not qd
                self.log(solver.t[i], row[:self.dof_count])

            state, t_init = self.state_machine.switch_to_next_state(solver)

    def log(self, time, q):
        """
        write a time sequence of generalized coordinates in csv file which is calculated by the ivp_solver
        :param time:
        :param q:
        :return:
        """
        if self.csv is not None:
            # define the string we want to put into the csv file which is the time t and the value of the generalized cooridnates q for this time stamp
            self.csv.write(str(time) + ", " + ", ".join(map(str, q)) + "\n")

        if self.ffcsv is not None:
            force = self.state_machine.math_model.ff
            #scale = abs(max(ff[3:6].min(), ff[3:6].max(), key=abs))
            #if scale != 0:
            #    self.state_machine.math_model.ff[3:6] = (ff[3:6] * 200 / scale)#scaled to 200
            self.ffcsv.write(str(time) + ", "+ ", ".join(map(str, force)) + "\n")

    def meshup(self):
        """
        creates the vision for the leg
        """
        if self.csv is not None:
            self.csv.close()
        # open meshup from python
        #os.system("meshup " + basefolder + "/articulatedLeg.lua " + basefolder + "/animation.csv")


if __name__ == "__main__":
    des_com_pos = 1  # one dimensional position

    model = ArticulatedLegWalker(
        leg_model_path="/home/stefan/git/dodo-alive-project/own-project-based-on-gruppe1/articulated_leg.lua",#basefolder + "/articulatedLeg.lua",
        des_com_pos=des_com_pos
    )

    # q : The model's initial position (x_cog, y_cog) and angles between links (J1, J2) i set
    # qd: The model's initial velocity is 0.
    q_init = np.array([0.0, 2.5, np.deg2rad(55), np.deg2rad(-100)])
    qd_init = np.array([0, 0, 0, 0])
    init_state = np.concatenate((q_init, qd_init))

    #initial and final time step for integrator
    t_final = 1

    model.state_machine.log_motion(t_final, init_state, model.log)
    #model.solve(t_init, t_final_state, init_state)
    model.meshup()
    model.robotPlotter.playbackMode()
