# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class BlockbeamSSIController(ControllerBase):
    def __init__(self, separate_integrator=True):
        # tuning parameters
        # tr_theta = 0.5  # value before tuning for rise time
        tr_theta = 0.25  # anything faster causes instability due to unmodeled nonlinear dynamics,
        # could also play with "M" to see if that helps.
        zeta_theta = 0.707
        M = 10
        tr_z = tr_theta * M
        zeta_z = 0.707
        integrator_pole = [-5.0]

        # augmented system
        A1 = np.block([[P.A, np.zeros((4, 1))], [-P.Cr, np.zeros(1)]])
        B1 = np.vstack((P.B, 0))

        # check controllability
        if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 5:
            raise ValueError("System not controllable")

        # compute gains
        wn_theta = 0.5 * np.pi / (tr_theta * np.sqrt(1 - zeta_theta**2))
        theta_char_poly = [1, 2 * zeta_theta * wn_theta, wn_theta**2]
        theta_poles = np.roots(theta_char_poly)

        wn_z = 0.5 * np.pi / (tr_z * np.sqrt(1 - zeta_z**2))
        z_char_poly = [1, 2 * zeta_z * wn_z, wn_z**2]
        z_poles = np.roots(z_char_poly)

        des_sys_poles = np.hstack([theta_poles, z_poles])
        des_poles = np.hstack((des_sys_poles, integrator_pole))

        self.K1 = cnt.place(A1, B1, des_poles)
        self.K = self.K1[:, :4]
        self.ki = self.K1[:, 4:]
        print("des_poles:", des_poles)
        print("K1:", self.K1)
        print("K:", self.K)
        print("ki:", self.ki)

        # linearization point
        self.x_eq = P.x_eq
        self.r_eq = P.Cr @ self.x_eq
        self.u_eq = P.u_eq
        self.x1_eq = np.hstack((self.x_eq, 0))

        # dirty derivative variables
        sigma = 0.05  # cutoff freq for dirty derivative
        self.beta = (2 * sigma - P.ts) / (2 * sigma + P.ts)
        self.zdot_hat = P.zdot0
        self.z_prev = P.z0
        self.thetadot_hat = P.thetadot0
        self.theta_prev = P.theta0

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

        # convert back to original variables
        u_unsat = u_tilde + self.u_eq
        u = self.saturate(u_unsat, u_max=P.force_max)
        return u