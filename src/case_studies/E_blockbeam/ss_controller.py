# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class BlockbeamSSController(ControllerBase):
    def __init__(self):
        # tuning parameters
        # tr_theta = 0.5  # value before tuning for rise time
        tr_theta = 0.25  # anything faster causes instability due to unmodeled nonlinear dynamics,
        # could also play with "M" to see if that helps.
        zeta_theta = 0.707
        M = 10
        tr_z = tr_theta * M
        zeta_z = 0.707

        # check controllability
        if np.linalg.matrix_rank(cnt.ctrb(P.A, P.B)) != 4:
            raise ValueError("System not controllable")

        # compute gains
        wn_theta = 0.5 * np.pi / (tr_theta * np.sqrt(1 - zeta_theta**2))
        theta_char_poly = [1, 2 * zeta_theta * wn_theta, wn_theta**2]
        theta_poles = np.roots(theta_char_poly)

        wn_z = 0.5 * np.pi / (tr_z * np.sqrt(1 - zeta_z**2))
        z_char_poly = [1, 2 * zeta_z * wn_z, wn_z**2]
        z_poles = np.roots(z_char_poly)

        des_poles = np.hstack([theta_poles, z_poles])

        self.K = cnt.place(P.A, P.B, des_poles)
        self.kr = -1.0 / (P.Cr @ np.linalg.inv(P.A - P.B @ self.K) @ P.B)
        print("des_poles:", des_poles)
        print("K:", self.K)
        print("kr:", self.kr)

        # linearization point
        self.x_eq = P.x_eq
        self.r_eq = P.Cr @ self.x_eq
        self.u_eq = P.u_eq

        # dirty derivative variables
        sigma = 0.05  # cutoff freq for dirty derivative
        self.beta = (2 * sigma - P.ts) / (2 * sigma + P.ts)
        self.zdot_hat = P.zdot0
        self.z_prev = P.z0
        self.thetadot_hat = P.thetadot0
        self.theta_prev = P.theta0

    def update_with_state(self, r, x):
        # convert to linearization (tilde) variables
        x_tilde = x - self.x_eq
        r_tilde = r - self.r_eq

        # compute state feedback control
        u_tilde = -self.K @ x_tilde + self.kr @ r_tilde

        # convert back to original variables
        u_unsat = u_tilde + self.u_eq
        u = self.saturate(u_unsat, u_max=P.force_max)
        return u
