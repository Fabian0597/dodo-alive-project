import math

import numpy as np
from numpy.linalg import inv

import sys
import pathlib

basefolder = str(pathlib.Path(__file__).parent.absolute())
sys.path.append(basefolder + '/../../rbdl-tum/build/python/')
import rbdl
import logging


def calc_numerical_gradient(x_new, x_old, step_size):
    if x_old is None:
        return np.zeros(np.shape(x_new))
    return (x_new - x_old) / step_size


class State:
    def __init__(self, q_size: int, q=None, qd=None, qdd=None):
        self.q = q or np.zeros(q_size)
        self.qd = qd or np.zeros(q_size)
        self.qdd = qdd or np.zeros(q_size)

    @classmethod
    def from_q_qd_array(cls, y, q_size):
        state = State(q_size)
        state.q = y[:q_size]
        state.qd = y[q_size:]
        return state

    def to_q_qd_array(self):
        return np.concatenate((self.q, self.qd), axis=0)


class MathModel:
    """
    represents the mathematical model, where all the variables of the robot leg are stored
    """

    def __init__(self, model, des_com_pos):
        self.model = model

        #for slip model
        self.slip_length = 0.6 #(used)
        self.slip_stiffness = 10000.  # TODO: 10000 #(used)
        self.ff = np.zeros(3)  # slip force #(used)





