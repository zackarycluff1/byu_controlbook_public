# 3rd-party
import numpy as np

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class RodMassControllerPID(ControllerBase):
    """
    PID controller for the rod-mass system.
    
    Can be used as PD controller (ki=0) or full PID controller.
    """

    def __init__(self, kp=0., kd=0., ki=0.):
        """
        Initialize PID controller with specified gains.
        
        Args:
            kp: Proportional gain
            kd: Derivative gain 
            ki: Integral gain
        """

        # TODO: initialize necessary variables for your controller here ...

    # TODO: implement functions needed for your controller here ...


