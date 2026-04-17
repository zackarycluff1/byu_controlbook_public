# 3rd-party
import numpy as np

# local (controlbook)
from . import params as P
from ..common.dynamics_base import DynamicsBase

from case_studies.F_vtol.eom_generated import calculate_eom

class VtolDynamics(DynamicsBase):
    def __init__(self, alpha=0.0):
        super().__init__(
            # Initial state conditions
            state0=np.array([P.z0, P.h0, P.theta0, P.zdot0, P.hdot0, P.thetadot0]),
            # Input limits
            u_max=P.force_max,
            u_min=-P.force_max,
            # Time step for integration
            dt=P.ts,
        )
        # see params.py/textbook for details on these parameters

        # parameter randomization is used after Chapter TODO:
        # you do not need to include it before then
        self.m1 = self.randomize_parameter(P.m1, alpha)
        self.m2 = self.randomize_parameter(P.m2, alpha)
        self.m3 = self.randomize_parameter(P.m3, alpha)
        self.g = self.randomize_parameter(P.g, alpha)
        self.d = self.randomize_parameter(P.d, alpha)
        self.Jc = self.randomize_parameter(P.Jc, alpha)
        self.m_c = self.randomize_parameter(P.mc, alpha)
        self.m_r = self.randomize_parameter(P.mr, alpha)
        self.b = self.randomize_parameter(P.b, alpha)

    def f(self, x, u):
        xdot = calculate_eom(x, u, self.m1, self.m2, self.m3, self.d, self.b, self.Jc)

        # F.14 disturbances
        wind = 1.0
        altitude_dist = 1.0
        xdot[0] += wind           # horizontal velocity disturbance
        xdot[4] += altitude_dist  # altitude acceleration disturbance

        return xdot

    def h(self):
        """
        Return the output y = h(x).
        """
        force, torque = self.state[:2]
        y = np.array([force, torque])  # measure position states only
        return y
