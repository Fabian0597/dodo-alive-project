import sys
import pathlib

basefolder = str(pathlib.Path(__file__).parent.absolute())
sys.path.append(basefolder + '/../../rbdl-tum/build/python/')
import rbdl

import numpy as np
from scipy.integrate import solve_ivp
import os
from gui import robot_plotter

from motion_hybrid_automaton.motion_hybrid_automaton import MotionHybridAutomaton
import logging
import sys

root = logging.getLogger()
root.setLevel(logging.DEBUG)

handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.INFO)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(module)s - %(message)s')
handler.setFormatter(formatter)
root.addHandler(handler)


class ModelSimulation:
    def __init__(self, leg_model_path, des_com_pos, show_gui):
        # instance of rbdl model is loaded
        # flight model

        logging.debug("Loading rbdl leg model")
        self.leg_model = rbdl.loadModel(leg_model_path)

        # get size of the generalized coordinates
        self.dof_count = self.leg_model.qdot_size

        logging.debug("set constraints")

        # stance phase
        self.constraint_set_stance = rbdl.ConstraintSet()
        # foot is constrained to move in the x plane and in the y plane (can not move anymore - is fixed to the ground)
        x_plane = np.array([1, 0, 0], dtype=np.double)
        y_plane = np.array([0, 1, 0], dtype=np.double)
        self.constraint_set_stance.AddContactConstraint(self.leg_model.GetBodyId("foot"), np.zeros(3), x_plane)
        self.constraint_set_stance.AddContactConstraint(self.leg_model.GetBodyId("foot"), np.zeros(3), y_plane)
        self.constraint_set_stance.Bind(self.leg_model)


        # flight phase
        self.constraint_set_flight = rbdl.ConstraintSet()

        """
        self.constraint_set_flight.AddContactConstraint(self.leg_model.GetBodyId("floatingBase"), np.zeros(3), x_plane)
        self.constraint_set_flight.AddContactConstraint(self.leg_model.GetBodyId("floatingBase"), np.zeros(3), y_plane)
        """

        self.constraint_set_flight.Bind(self.leg_model)


        logging.debug("init gui robot plotter")
        self.show_gui = show_gui
        if show_gui:
            self.robot_plotter = robot_plotter.RobotPlotter(self.leg_model, None)
            self.robot_plotter.setAxisLimit((-1, 6), (-1, 4))
            for el in ["floatingBase", "link1", "link2", "foot"]:
                self.robot_plotter.addBodyId(self.leg_model.GetBodyId(el))
            self.robot_plotter.showPoints(0, "k-", [-1000, 0], [1000, 0])  # plot ground
        else:
            self.robot_plotter = None

        logging.debug("open csv file")
        self.csv = open(basefolder + '/animation.csv', 'w')
        self.ffcsv = open(basefolder + '/forces.ff', 'w')

        logging.debug("init Motion State Machine")
        self.state_machine = MotionHybridAutomaton(self.leg_model, des_com_pos, self.constraint_set_flight, self.constraint_set_stance,
                                                   self.robot_plotter)

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
            force = self.state_machine.slip_model.slip_force
            # scale = abs(max(ff[3:6].min(), ff[3:6].max(), key=abs))
            # if scale != 0:
            #    self.state_machine.math_model.ff[3:6] = (ff[3:6] * 200 / scale)#scaled to 200
            self.ffcsv.write(str(time) + ", " + ", ".join(map(str, force)) + "\n")

    def visualize(self):
        """
        creates the vision for the leg
        """
        if self.csv is not None:
            self.csv.close()

        if self.show_gui:
            self.robot_plotter.playbackMode()

        # open meshup from python
        os.system("meshup " + basefolder + "/model/articulatedLeg.lua " + basefolder + "/animation.csv")


if __name__ == "__main__":
    des_com_pos = 0  # one dimensional goal position of center of mass (com)

    model = ModelSimulation(
        leg_model_path=basefolder + "/model/articulatedLeg.lua",
        des_com_pos=des_com_pos,
        show_gui=False,
    )

    # q : The model's initial position (x_cog, y_cog) and angles between links (J1, J2) i set
    # qd: The model's initial velocity is 0.
    q_init = np.array([0.0, 1.5, 0.01, 0.01])# np.deg2rad(55), np.deg2rad(-100)])
    qd_init = np.array([0, 0, 0, 0])
    init_state = np.concatenate((q_init, qd_init))

    # initial and final time step for integrator
    t_final = 5

    model.state_machine.simulate_motion(t_final, init_state, model.log)

    model.visualize()
