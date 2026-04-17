# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class RodMassLQRController(ControllerBase):
    """
    LQR-based state-space integral control with disturbance observer for rod-mass system.
    
    Uses:
    - LQR (Linear Quadratic Regulator) for optimal gain selection
    - Integral action for steady-state error elimination
    - Disturbance observer to estimate and reject constant disturbances
    """

    def __init__(self, separate_integrator=True):
        """
        Initialize LQR controller with tunable Q and R matrices.
        """
        # TODO: initialize necessary variables for your controller here ...
    # TODO: implement functions needed for your controller here ...
