# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class MassSSController(ControllerBase):
    def __init__(self):
        # tuning parameters
        tr = 2
        zeta = 0.707

        # check controllability
        if np.linalg.matrix_rank(cnt.ctrb(P.A, P.B)) != 2:
            raise ValueError("System not controllable")

        # compute gains
        wn = 2.2 / tr  # assumes zeta = 0.707
        des_char_poly = [1, 2 * zeta * wn, wn**2]
        des_poles = np.roots(des_char_poly)
        self.K = cnt.place(P.A, P.B, des_poles)
        self.kr = -1.0 / (P.Cr @ np.linalg.inv(P.A - P.B @ self.K) @ P.B)
        print("des_poles:", des_poles)
        print("K:", self.K)
        print("kr:", self.kr)

        # linearization point
        self.x_eq = P.x_eq
        self.r_eq = P.Cr @ self.x_eq

        # dirty derivative variables
        sigma = 0.05  # cutoff freq for dirty derivative
        self.beta = (2 * sigma - P.ts) / (2 * sigma + P.ts)
        self.zdot_hat = P.zdot0
        self.z_prev = P.z0

    def update_with_state(self, r, x):
        # convert to linearization (tilde) variables
        x_tilde = x - self.x_eq
        r_tilde = r - self.r_eq

        # compute state feedback control
        u_tilde = -self.K @ x_tilde + self.kr @ r_tilde

        # convert back to original variables (feedback linearization)
        z = x[0]
        u_fl = 0
        u_unsat = u_tilde + u_fl
        u = self.saturate(u_unsat, u_max=P.force_max)
        return u
