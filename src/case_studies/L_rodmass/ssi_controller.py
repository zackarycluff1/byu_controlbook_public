# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class RodMassSSIController(ControllerBase):
    """
    State-space integral control for rod-mass system. 
    
    Uses:
    - Full state feedback with integral action
    - No observer (this is for Part 4.1-4.2 of practice final)
    """

    def __init__(self, separate_integrator=True):
        """
        Initialize controller with pole placement design.
        
        """

        # TODO: initialize necessary variables for your controller here ...

    # TODO: implement functions needed for your controller here ...
