# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class MassSSIController(ControllerBase):
    def __init__(self, separate_integrator=True):
        # tuning parameters
        tr = 2
        zeta = 0.707
        integrator_pole = [-5.0]

        # augmented system
        A1 = np.block([[P.A, np.zeros((2, 1))], [-P.Cr, np.zeros(1)]])
        B1 = np.vstack((P.B, 0))

        # check controllability
        if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 3:
            raise ValueError("System not controllable")

        # compute gains
        wn = 2.2 / tr  # assumes zeta = 0.707
        des_char_poly = [1, 2 * zeta * wn, wn**2]
        des_sys_poles = np.roots(des_char_poly)
        des_poles = np.hstack((des_sys_poles, integrator_pole))
        self.K1 = cnt.place(A1, B1, des_poles)
        self.K = self.K1[:, :2]
        self.ki = self.K1[:, 2:]
        print("des_poles:", des_poles)
        print("K1:", self.K1)
        print("K:", self.K)
        print("ki:", self.ki)

        # linearization point
        self.x_eq = P.x_eq
        self.r_eq = P.Cr @ self.x_eq
        self.x1_eq = np.hstack((self.x_eq, 0))

        # dirty derivative variables
        sigma = 0.05  # cutoff freq for dirty derivative
        self.beta = (2 * sigma - P.ts) / (2 * sigma + P.ts)
        self.zdot_hat = P.zdot0
        self.z_prev = P.z0

        # integrator variables
        self.error_prev = 0.0
        self.error_integral = 0.0
        self.separate_integrator = separate_integrator

    def update_with_state(self, r, x):
        # convert to linearization (tilde) variables
        x_tilde = x - self.x_eq

        # integrate error
        error = r - P.Cr @ x  # can also use tilde vars (eq subtracts out)
        self.error_integral += P.ts * (error + self.error_prev) / 2
        self.error_prev = error

        # compute feedback control
        if self.separate_integrator:
            u_tilde = -self.K @ x_tilde - self.ki @ self.error_integral
        else:
            x1_tilde = np.hstack((x_tilde, self.error_integral))
            u_tilde = -self.K1 @ x1_tilde

        # convert back to original variables (feedback linearization)
        z = x[0]
        u_fl = 0
        u_unsat = u_tilde + u_fl
        u = self.saturate(u_unsat, u_max=P.force_max)
        return u