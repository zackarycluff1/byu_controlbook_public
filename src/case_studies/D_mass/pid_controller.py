# 3rd-party
import numpy as np

# local (controlbook)
from . import params as P
from .. import common


class MassControllerPID(common.ControllerBase):
    def __init__(self, ki):
        # tuning parameters
        tr = 2
        zeta = 0.707
        self.ki = ki

        # system parameters
        b0 = P.tf_num[-1]
        a1, a0 = P.tf_den[-2:]

        # find gains
        wn = 0.5 * np.pi / (tr * np.sqrt(1 - zeta**2))
        alpha0 = wn**2
        alpha1 = 2 * zeta * wn
        self.kp = (alpha0 - a0) / b0
        self.kd = (alpha1 - a1) / b0
        print(f"{self.kp = :.3f}, {self.ki = :.3f}, {self.kd = :.3f}")

        self.u_eq = P.u_eq

        # variables for dirty derivative
        self.sigma = 0.05
        self.beta = (2 * self.sigma - P.ts) / (2 * self.sigma + P.ts)
        self.zdot_hat = P.zdot0  # estimated derivative of z
        self.z_prev = P.z0

        # variables for integrator
        self.error_prev = 0.0
        self.error_integral = 0.0

    def update_with_measurement(self, r, y):
        z = y[0]
        z_ref = r[0]

        # dirty derivative to estimate thetadot
        z_diff = (z - self.z_prev) / P.ts
        self.zdot_hat = self.beta * self.zdot_hat + (1 - self.beta) * z_diff
        self.z_prev = z

        # compute input from partially estimated state
        xhat = np.array([z, self.zdot_hat])

        # integrate error
        error = z_ref - z
        if (
            abs(self.zdot_hat) < 0.08
        ):  # anti-windup: only integrate if thetadot is small
            self.error_integral += P.ts * (error + self.error_prev) / 2
        self.error_prev = error

        # theta (modified) PID
        F_tilde = (
            self.kp * error
            + self.ki * self.error_integral
            - self.kd * self.zdot_hat
        )
        f_fl = 0
        force = F_tilde + f_fl
        u_unsat = np.array([force])
        u = self.saturate(u_unsat, u_max=P.force_max)

        return u, xhat
