# 3rd-party
import array

import numpy as np
from numpy.typing import NDArray

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class RodMassEquilibrium(ControllerBase):
    """
    PID controller for the rod-mass system.
    
    Can be used as PD controller (ki=0) or full PID controller.
    """

    def __init__(self):
        """
        Initialize PID controller with specified gains.
        
        Args:
            kp: Proportional gain
            kd: Derivative gain 
            ki: Integral gain
        """
        self.u_e = np.array([P.m*P.g*P.ell])

        # TODO: initialize necessary variables for your controller here ...

    # TODO: implement functions needed for your controller here ...
    def update_with_measurement(self, r, y):
        theta = y[0]
        thetadot = 0
        x = np.array([theta, thetadot])
        u = self.u_e
        return u, x


