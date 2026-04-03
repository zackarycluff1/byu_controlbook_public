"""
═══════════════════════════════════════════════════════════════════════════════
CONTROLLER TEMPLATE FOR STUDENTS
═══════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
from ..common import ControllerBase
from . import params as P


class HummingbirdControllerFullPD(ControllerBase):
    def __init__(self):

        th_zeta = 0.707
        psi_zeta = 0.9
        phi_zeta = 0.707
        
        # Longitudinal (theta) gains
        tr_theta = 0.4
        wn_theta = 2.2 / tr_theta
        alpha1_theta = 2 * th_zeta * wn_theta
        alpha0_theta = wn_theta**2

        # Roll (phi) gains
        tr_phi = 0.1
        wn_phi = 2.2 / tr_phi
        alpha1_phi = 2 * phi_zeta * wn_phi
        alpha0_phi = wn_phi**2

        # Yaw (psi) gains
        tr_psi = 0.8
        wn_psi = 2.2 / tr_psi
        alpha1_psi = 2 * psi_zeta * wn_psi
        alpha0_psi = wn_psi**2

        # Transfer function coefficients from params
        b0_theta = P.tf_theta_num[-1]
        a1_theta, a0_theta = P.tf_theta_den[-2:]
        self.kp_theta = (alpha0_theta - a0_theta) / b0_theta
        self.kd_theta = (alpha1_theta - a1_theta) / b0_theta
        print(f"Theta PD Gains: kp = {self.kp_theta:.4f}, kd = {self.kd_theta:.4f}")

        b0_phi = P.tf_phi_num[-1]
        a1_phi, a0_phi = P.tf_phi_den[-2:]
        self.kp_phi = (alpha0_phi - a0_phi) / b0_phi
        self.kd_phi = (alpha1_phi - a1_phi) / b0_phi
        print(f"Phi PD Gains: kp = {self.kp_phi:.4f}, kd = {self.kd_phi:.4f}")

        b0_psi = P.tf_psi_num[-1]
        a1_psi, a0_psi = P.tf_psi_den[-2:]
        self.kp_psi = (alpha0_psi - a0_psi) / b0_psi
        self.kd_psi = (alpha1_psi - a1_psi) / b0_psi
        print(f"Psi PD Gains: kp = {self.kp_psi:.4f}, kd = {self.kd_psi:.4f}")

        # Integral gains (tune these)
        self.ki_theta = 0.5
        self.ki_psi = 0.4

        # Integral accumulators
        self.integral_theta_error = 0.0
        self.integral_psi_error = 0.0

        # Previous errors for trapezoidal integration
        self.error_theta_prev = 0.0
        self.error_psi_prev = 0.0

        # Anti-windup limits
        self.theta_integrator_limit = 0.5
        self.psi_integrator_limit = 1.0

        # Low-pass filter for numerical derivatives
        self.sigma = 0.05
        self.beta = (2 * self.sigma - P.ts) / (2 * self.sigma + P.ts)

        # Previous values for derivative estimates
        self.phi_prev = P.phi0
        self.theta_prev = 0.0
        self.psi_prev = P.psi0

        # Derivative estimates
        self.phidot_hat = P.phidot0
        self.thetadot_hat = 0.0
        self.psidot_hat = P.psidot0

        self.u_eq = P.km

    def update_with_measurement(self, r, y):

        # ═══════════════════════════════════════
        # STEP 1: Unpack reference and measurement
        # ═══════════════════════════════════════
        theta_ref = r[1]
        psi_ref = r[2]

        phi, theta, psi = y

        # ═══════════════════════════════════════
        # STEP 2: Numerical derivatives (low-pass filtered)
        # ═══════════════════════════════════════
        theta_diff = (theta - self.theta_prev) / P.ts
        self.thetadot_hat = self.beta * self.thetadot_hat + (1 - self.beta) * theta_diff
        self.theta_prev = theta

        psi_diff = (psi - self.psi_prev) / P.ts
        self.psidot_hat = self.beta * self.psidot_hat + (1 - self.beta) * psi_diff
        self.psi_prev = psi

        phi_diff = (phi - self.phi_prev) / P.ts
        self.phidot_hat = self.beta * self.phidot_hat + (1 - self.beta) * phi_diff
        self.phi_prev = phi

        # ═══════════════════════════════════════
        # STEP 3: Longitudinal (theta) PID
        # ═══════════════════════════════════════
        theta_error = theta_ref - theta

        # Trapezoidal integration
        if abs(self.thetadot_hat) < 0.1:
            self.integral_theta_error += P.ts * (theta_error + self.error_theta_prev) / 2
        self.error_theta_prev = theta_error

        # Anti-windup clamp
        self.integral_theta_error = np.clip(
            self.integral_theta_error,
            -self.theta_integrator_limit,
            self.theta_integrator_limit
        )

        # PID control law
        f_tilde = (
            self.kp_theta * theta_error
            + self.ki_theta * self.integral_theta_error
            - self.kd_theta * self.thetadot_hat
        )

        # ═══════════════════════════════════════
        # STEP 4: Outer lateral (psi) PID -> phi_ref
        # ═══════════════════════════════════════
        psi_error = psi_ref - psi

        # Trapezoidal integration
        if abs(self.psidot_hat) < 0.1:
            self.integral_psi_error += P.ts * (psi_error + self.error_psi_prev) / 2
        self.error_psi_prev = psi_error

        # Anti-windup 
        self.integral_psi_error = np.clip(
            self.integral_psi_error,
            -self.psi_integrator_limit,
            self.psi_integrator_limit
        )

        # Outer loop PID produces phi reference
        phi_ref = (
            self.kp_psi * psi_error
            + self.ki_psi * self.integral_psi_error
            - self.kd_psi * self.psidot_hat
        )

        # ═══════════════════════════════════════
        # STEP 5: Inner lateral (phi) PD
        # ═══════════════════════════════════════
        phi_error = phi_ref - phi
        tau_tilde = (
            self.kp_phi * phi_error
            - self.kd_phi * self.phidot_hat
        )

        # ═══════════════════════════════════════
        # STEP 6: Feedback linearization + total force
        # ═══════════════════════════════════════
        F_fl = (P.m1 * P.ell_1 + P.m2 * P.ell_2) * P.g / P.ell_T * np.cos(theta)
        u_total = f_tilde + F_fl

        # ═══════════════════════════════════════
        # STEP 7: Mix, saturate, and return
        # ═══════════════════════════════════════
        u = P.mixer @ np.array([u_total, tau_tilde])
        u = self.saturate(u, u_max=50)

        xhat = np.array([phi, theta, psi, self.phidot_hat, self.thetadot_hat, self.psidot_hat])

        return u, xhat