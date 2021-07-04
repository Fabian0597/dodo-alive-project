import numpy as np


class SlipModelParameterization:
    """
    represents the mathematical model, where all the variables of the robot leg are stored
    """

    def __init__(self):
        # for slip model
        self.slip_length_zero = 0.6
        self.slip_length = 0.6  # (used)
        self.slip_stiffness = 10000.  # TODO: 10000 #(used)
        self.slip_force = np.zeros(3)  # slip force #(used)
