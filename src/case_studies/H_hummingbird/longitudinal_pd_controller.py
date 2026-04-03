import numpy as np
 
# local (controlbook)
from ..common import ControllerBase
from . import params as P
 
 
class HummingbirdControllerLonPD(ControllerBase):
    """
    PD controller for the longitudinal (pitch) dynamics of the hummingbird.
 
    The linearized longitudinal model (eq. 4.3) is:
        theta_ddot = b_theta * F_tilde
 
    where b_theta = ell_T / (m1*ell_1^2 + m2*ell_2^2 + J1y + J2y)
 
    The closed-loop characteristic equation with PD control is:
        s^2 + kd*b_theta*s + kp*b_theta = 0
 
    Gains are found by matching coefficients to the desired polynomial:
        s^2 + 2*zeta*wn*s + wn^2 = 0
    """
 
    def __init__(self):
        # ═══════════════════════════════════════════════════════════════
        # Design parameters - tune these to change response
        # ═══════════════════════════════════════════════════════════════
        tr = 1.0      # desired rise time (seconds)
        zeta = 0.707  # damping ratio
 
        wn = 2.2 / tr        # natural frequency from rise time: tr ≈ 2.2 / wn
        alpha1 = 2 * zeta * wn
        alpha0 = wn**2
 
        # ═══════════════════════════════════════════════════════════════
        # Transfer function coefficient b_theta (eq. 4.4)
        # G(s) = b_theta / s^2,  so a1=0, a0=0, b0=b_theta
        # ═══════════════════════════════════════════════════════════════
        b0 = P.ell_T / (P.m1 * P.ell_1**2 + P.m2 * P.ell_2**2 + P.J1y + P.J2y)
        a1 = 0.0
        a0 = 0.0
 
        # ═══════════════════════════════════════════════════════════════
        # PD gains from coefficient matching:
        #   s^2 + (a1 + b0*kd)*s + (a0 + b0*kp) = s^2 + alpha1*s + alpha0
        # ═══════════════════════════════════════════════════════════════
        self.kp = (alpha0 - a0) / b0
        self.kd = (alpha1 - a1) / b0
        print(f"Longitudinal PD Gains: kp = {self.kp:.4f}, kd = {self.kd:.4f}")
 
        # ═══════════════════════════════════════════════════════════════
        # Low-pass filter for derivative (Tustin discretization of
        # H(s) = s / (sigma*s + 1))
        # ═══════════════════════════════════════════════════════════════
        self.sigma = 0.05
        self._a1_filt = (2.0 * self.sigma - P.ts) / (2.0 * self.sigma + P.ts)
        self._b0_filt = 2.0 / (2.0 * self.sigma + P.ts)
        self._b1_filt = -2.0 / (2.0 * self.sigma + P.ts)
        self._theta_prev = 0.0
        self._thetadot_prev = 0.0
 
        # ═══════════════════════════════════════════════════════════════
        # Equilibrium force (feedback linearization cancels gravity)
        #   F_fl = (m1*ell_1 + m2*ell_2) * g / ell_T * cos(theta)
        # ═══════════════════════════════════════════════════════════════
        self.u_eq = (P.m1 * P.ell_1 + P.m2 * P.ell_2) * P.g / P.ell_T
 
    def update_with_state(self, r, x):
        """
        Compute motor forces [f_l, f_r] using PD pitch control.
 
        Args:
            r: Reference vector - [theta_ref] (radians)
            x: State vector - [phi, theta, psi, phidot, thetadot, psidot]
 
        Returns:
            u: np.ndarray [f_l, f_r] motor forces (Newtons)
        """
        # ═══════════════════════════════════════════════════════════════
        # STEP 1: Unpack reference and state
        # ═══════════════════════════════════════════════════════════════
        theta_ref = r[1]
 
        theta = x[1]
        thetadot = x[4]
 
        # ═══════════════════════════════════════════════════════════════
        # STEP 2: Compute error
        # ═══════════════════════════════════════════════════════════════
        error = theta_ref - theta
 
        # ═══════════════════════════════════════════════════════════════
        # STEP 3: Apply PD control law
        # ═══════════════════════════════════════════════════════════════
        # Feedback linearization force cancels the gravity nonlinearity
        F_fl = (P.m1 * P.ell_1 + P.m2 * P.ell_2) * P.g / P.ell_T * np.cos(theta)
 
        # PD deviation force
        F_tilde = self.kp * error - self.kd * thetadot
 
        # Total force and zero torque (roll/yaw locked for this lab)
        F_cmd = F_fl + F_tilde
        tau_cmd = 0.0
 
        # ═══════════════════════════════════════════════════════════════
        # STEP 4: Convert [F, tau] -> [f_l, f_r] and return
        # ═══════════════════════════════════════════════════════════════
        u = P.mixer @ np.array([F_cmd, tau_cmd])
        u = self.saturate(u, u_max=np.inf)  # clip to non-negative (no pulling)
        u = np.clip(u, 0.0, np.inf)
 
        return u