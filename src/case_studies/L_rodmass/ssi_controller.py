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
        tr = 0.25
        zeta = 0.95
        
        # compute gains
        wn = 0.5 * np.pi / (tr * np.sqrt(1 - zeta**2))
        des_char_poly = [1, 2 * zeta * wn, wn**2]
        des_poles = np.roots(des_char_poly)
        self.K = cnt.place(P.A, P.B, des_poles)
        self.kr = -1.0 / (P.Cr @ np.linalg.inv(P.A - P.B @ self.K) @ P.B)
        print("des_poles:", des_poles)
        print("K:", self.K)
        print("kr:", self.kr)
    # TODO: implement functions needed for your controller here ...
