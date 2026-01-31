# 3rd-party
import numpy as np

# local (controlbook)
from . import params as P
from ..common.dynamics_base import DynamicsBase

from case_studies.D_mass.eom_generated import calculate_eom

class MassDynamics(DynamicsBase):
    def __init__(self, alpha=0.0):
        super().__init__(
            # Initial state conditions
            state0=np.array([P.z0, P.zdot0]),
            # Input force limits
            u_max=P.force_max,
            u_min=-P.force_min,
            # Time step for integration
            dt=P.ts,
        )
        # see params.py/textbook for details on these parameters
        self.m = self.randomize_parameter(P.m, alpha)
        self.k = self.randomize_parameter(P.k, alpha)
        self.b = self.randomize_parameter(P.b, alpha)
        self.g = P.g  # gravity constant is well known, so not randomized

    def f(self, x, u):
        # Return xdot = f(x,u), the system state update equations
        # re-label states for readability
        
        xdot = calculate_eom(x, u, self.m, self.k, self.b)
        return xdot

    def h(self):
        # return the output equations
        # could also use input u if needed
        z = self.state[0]
        y = np.array([z])
        return y
