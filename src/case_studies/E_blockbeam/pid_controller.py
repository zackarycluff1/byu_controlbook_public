# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common
from case_studies.E_blockbeam import params as P


class BlockbeamControllerPID(common.ControllerBase):
    def __init__(self):
        ## control tuning parameters
        self.ki_z = -0.005  # TODO: works better with 0 ... or -0.01
        zeta = 0.707

        M = 10

        tr_th = 0.15
        wn_th = 2.2 / tr_th
        alpha1_th = 2 * zeta * wn_th
        alpha0_th = wn_th**2

        tr_z = M * tr_th
        wn_z = 2.2 / tr_z
        alpha1_z = 2 * zeta * wn_z
        alpha0_z = wn_z**2

        # ═══════════════════════════════════════════════════════════════
        # TODO: Get transfer function coefficients from params.py
        # ═══════════════════════════════════════════════════════════════
        # These come from your system's transfer function (Chapter 5):
        # G(s) = b0 / (s^2 + a1*s + a0)
        # ═══════════════════════════════════════════════════════════════
        # Theta Kd and Kp


        b0_th = P.tf_inner_num[-1]
        a1_th, a0_th = P.tf_inner_den[-2:]

        # Closed-loop: s^2 + (a1 + b0*kd)*s + (a0 + b0*kp) = s^2 + alpha1*s + alpha0
        self.kp_th = (alpha0_th - a0_th) / b0_th  # Proportional gain
        self.kd_th = (alpha1_th - a1_th) / b0_th  # Derivative gain
        print(f"Theta PD Gains: kp = {self.kp_th:.2f}, kd = {self.kd_th:.3f}")
        # ═══════════════════════════════════════════════════════════════

        # ═══════════════════════════════════════════════════════════════
        # Z Kd and Kp

        self.b0_z = P.tf_outer_num[-1]
        self.a1_z, self.a0_z = P.tf_outer_den[-2:]

        # Closed-loop: s^2 + (a1 + b0*kd)*s + (a0 + b0*kp) = s^2 + alpha1*s + alpha0
        self.kp_z = (alpha0_z - self.a0_z) / self.b0_z  # Proportional gain
        self.kd_z = (alpha1_z - self.a1_z) / self.b0_z  # Derivative gain
        print(f"Z PD Gains: kp = {self.kp_z:.2f}, kd = {self.kd_z:.3f}")

        # dirty derivative variables
        self.sigma = 0.05
        self.beta = (2 * self.sigma - P.ts) / (2 * self.sigma + P.ts)
        self.thetadot_hat = P.thetadot0
        self.theta_prev = P.theta0
        self.zdot_hat = P.zdot0
        self.z_prev = P.z0

        # integrator variables
        self.error_z_prev = 0.0
        self.integral_z_error = 0.0

        

    def update_with_measurement(self, r, y):
        z, theta = y
        z_ref = r[0]

        # dirty derivative to estimate zdot
        z_diff = (z - self.z_prev) / P.ts
        self.zdot_hat = self.beta * self.zdot_hat + (1 - self.beta) * z_diff
        self.z_prev = z

        # dirty derivative to estimate thetadot
        theta_diff = (theta - self.theta_prev) / P.ts
        self.thetadot_hat = self.beta * self.thetadot_hat + (1 - self.beta) * theta_diff
        self.theta_prev = theta

        # compute input from partially estimated state
        xhat = np.array([z, theta, self.zdot_hat, self.thetadot_hat])

        # outer loop control
        error_z = z_ref - z
        if abs(self.zdot_hat) < 0.08:  # anti-windup: only integrate if zdot is small
            self.integral_z_error += P.ts * (error_z + self.error_z_prev) / 2
        self.error_z_prev = error_z
        theta_ref = (
            self.kp_z * error_z
            + self.ki_z * self.integral_z_error
            - self.kd_z * self.zdot_hat
        )

        r[1] = theta_ref  # if you want to visualize the "reference" angle

        # inner loop control
        error_theta = theta_ref - theta
        F_fl = P.g*(2*z*P.m1 + P.m2*P.ell)/(2*P.ell)
        F = self.kp_th * error_theta - self.kd_th * self.thetadot_hat + F_fl
        u_unsat = np.array([F])
        u = self.saturate(u_unsat, u_max=P.force_max)

        return u, xhat
