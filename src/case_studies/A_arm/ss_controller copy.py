# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class ArmSSController(ControllerBase):
    def __init__(self):
        # tuning parameters
        tr = 0.489
        zeta = 0.707

        integral_of_error = 0
        
        # check controllability
        if np.linalg.matrix_rank(cnt.ctrb(P.A, P.B)) != 2:
            raise ValueError("System not controllable")

        # compute gains
        wn = 2.2 / tr  # assumes zeta = 0.707
        des_char_poly = [1, 2 * zeta * wn, wn**2]


        des_poles = np.roots(des_char_poly)

        des_x_I_pole = -0.5
        des_poles_final = np.hstack((des_poles, des_x_I_pole))

        A1 = np.zeros(P.A.shape[0]+1, P.A.shape[0]+1)
        A1[0:P.A.shape[0], 0:P.A.shape[0]]
        A1[-1,:-1] = -P.Cr
        B1 = np.vstack((P.B, 0.0))

        K1 = cnt.place(A1, B1, des_poles_final)

        self.K = cnt.place(P.A, P.B, des_poles)
        self.kr = -1.0 / (P.Cr @ np.linalg.inv(P.A - P.B @ self.K) @ P.B)
        print("des_poles:", des_poles)
        print("K:", self.K)
        print("kr:", self.kr)

        self.K=K1[0,:-1]
        self.ki = K1[0, -1]

        # linearization point
        self.x_eq = P.x_eq
        self.r_eq = P.Cr @ self.x_eq

        # dirty derivative variables
        sigma = 0.05  # cutoff freq for dirty derivative
        self.beta = (2 * sigma - P.ts) / (2 * sigma + P.ts)
        self.thetadot_hat = P.thetadot0
        self.theta_prev = P.theta0

    def update_with_state(self, r, x):
        # convert to linearization (tilde) variables
        x_tilde = x - self.x_eq
        r_tilde = r - self.r_eq

        # compute state feedback control
        u_tilde = -self.K @ x_tilde - self.ki @ integral_of_error

        # convert back to original variables (feedback linearization)
        theta = x[0]
        u_fl = P.m * P.g * P.ell / 2 * np.cos(theta)
        u_unsat = u_tilde + u_fl
        u = self.saturate(u_unsat, u_max=P.tau_max)
        return u
