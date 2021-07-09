import numpy as np


class SlipModelParameterization:
    """
    stores some parameters for the slip model
    """

    def __init__(self):
        # for slip model
        self.slip_length_zero = 0.9  # controls the desired leg length for the impact
        self.slip_length = None
        self.slip_stiffness = 10000.
        self.slip_force = np.zeros(3)  # slip force #(used)
