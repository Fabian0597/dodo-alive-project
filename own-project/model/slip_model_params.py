import numpy as np


def calc_numerical_gradient(x_new, x_old, step_size):
    if x_old is None:
        return np.zeros(np.shape(x_new))
    return (x_new - x_old) / step_size


class SlipModelParameterization:
    """
    represents the mathematical model, where all the variables of the robot leg are stored
    """

    def __init__(self):
        # for slip model
        self.slip_length = 0.6  # (used)
        self.slip_stiffness = 10000.  # TODO: 10000 #(used)
        self.slip_force = np.zeros(3)  # slip force #(used)
