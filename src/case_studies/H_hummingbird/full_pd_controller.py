import numpy as np

# local (controlbook)
from ..common import ControllerBase
from . import params as P


class HummingbirdControllerFullPD(ControllerBase):
    """
    Full PD controller combining:
      - Longitudinal (pitch) loop  — pole-placement PD as H.7
      - Lateral inner loop (roll)  — PD on phi
      - Lateral outer loop (yaw)   — PD on psi, output is phi reference

    Uses update_with_measurement(r, y) where y = [phi, theta, psi] (angles only).
    All derivatives are estimated using a Tustin low-pass filtered differentiator.

    Longitudinal model (eq. 4.3):
        theta_ddot = b_theta * F_tilde
        b_theta = ell_T / (m1*ell_1^2 + m2*ell_2^2 + J1y + J2y)

    Roll model (eq. 4.7):
        phi_ddot = (1/J1x) * tau_tilde

    Yaw model (eq. 4.8) — driven by phi:
        psi_ddot = b_psi * phi_tilde
        b_psi = Fe * ell_T / (J1z + J2z + J3z + ell_1^2*m1 + ell_2^2*m2
                               + ell_3x^2*m3 + ell_3y^2*m3)

    Successive loop closure:
        - Roll (inner): rise time  tr_phi = tr_psi / M
        - Yaw  (outer): rise time  tr_psi  (chosen by designer)
        - Bandwidth separation factor M = 10..20
    """

    def __init__(self):
        # ═══════════════════════════════════════════════════════════════
        # ── LONGITUDINAL (PITCH) design parameters ───────────────────
        # ═══════════════════════════════════════════════════════════════
        tr_theta   = 1.0    # desired rise time for pitch (s)
        zeta_theta = 0.707

        M = 10.0   
        tr_psi = 5.0   
        tr_phi = tr_psi / M

        zeta_phi = 0.707
        zeta_psi = 0.707

        wn_theta     = 2.2 / tr_theta
        alpha1_theta = 2.0 * zeta_theta * wn_theta
        alpha0_theta = wn_theta ** 2

        self.b_theta = P.ell_T / (P.m1 * P.ell_1**2 + P.m2 * P.ell_2**2
                             + P.J1y + P.J2y)

        self.kp_theta = alpha0_theta / self.b_theta
        self.kd_theta = alpha1_theta / self.b_theta
        print(f"Pitch  PD Gains : kp = {self.kp_theta:.4f}, "
              f"kd = {self.kd_theta:.4f}")

        self.sigma_theta = 0.05
        self._a1_theta  = (2.0 * self.sigma_theta - P.ts) / (2.0 * self.sigma_theta + P.ts)
        self._b0_theta  =  2.0 / (2.0 * self.sigma_theta + P.ts)
        self._b1_theta  = -2.0 / (2.0 * self.sigma_theta + P.ts)
        self._theta_prev    = 0.0
        self._thetadot_prev = 0.0
        self.thetadot_hat = P.thetadot0

        self.F_eq = (P.m1 * P.ell_1 + P.m2 * P.ell_2) * P.g / P.ell_T

        # ═══════════════════════════════════════════════════════════════
        # ── LATERAL design parameters ─────────────────────────────────
        # ═══════════════════════════════════════════════════════════════

        # ── Roll (inner) ──────────────────────────────────────────────
        wn_phi     = 2.2 / tr_phi
        alpha1_phi = 2.0 * zeta_phi * wn_phi
        alpha0_phi = wn_phi ** 2

        self.b_phi = 1.0 / P.J1x   # from eq 4.7: phi_ddot = tau / J1x

        self.kp_phi = alpha0_phi / self.b_phi
        self.kd_phi = alpha1_phi / self.b_phi
        print(f"Roll   PD Gains : kp = {self.kp_phi:.4f}, "
              f"kd = {self.kd_phi:.4f}")

        # Tustin derivative filter for phi
        self.sigma_phi = 0.05
        self._a1_phi  = (2.0 * self.sigma_phi - P.ts) / (2.0 * self.sigma_phi + P.ts)
        self._b0_phi  =  2.0 / (2.0 * self.sigma_phi + P.ts)
        self._b1_phi  = -2.0 / (2.0 * self.sigma_phi + P.ts)
        self._phi_prev    = 0.0
        self._phidot_prev = 0.0
        self.phidot_hat = P.phidot0

        # ── Yaw (outer) ───────────────────────────────────────────────
        wn_psi     = 2.2 / tr_psi
        alpha1_psi = 2.0 * zeta_psi * wn_psi
        alpha0_psi = wn_psi ** 2

        J_denom = (P.J1z + P.J2z + P.J3z
                   + P.ell_1**2  * P.m1
                   + P.ell_2**2  * P.m2
                   + P.ell_3x**2 * P.m3
                   + P.ell_3y**2 * P.m3)
        self.b_psi = self.F_eq * P.ell_T / J_denom

        self.kp_psi = alpha0_psi / self.b_psi
        self.kd_psi = alpha1_psi / self.b_psi
        print(f"Yaw    PD Gains : kp = {self.kp_psi:.4f}, "
              f"kd = {self.kd_psi:.4f}")

        # Tustin derivative filter for psi
        self.sigma_psi = 0.05
        self._a1_psi  = (2.0 * self.sigma_psi - P.ts) / (2.0 * self.sigma_psi + P.ts)
        self._b0_psi  =  2.0 / (2.0 * self.sigma_psi + P.ts)
        self._b1_psi  = -2.0 / (2.0 * self.sigma_psi + P.ts)
        self._psi_prev    = 0.0
        self._psidot_prev = 0.0
        self.psidot_hat = P.psidot0

    # ──────────────────────────────────────────────────────────────────
    def update_with_measurement(self, r, y):
        """
        Compute motor forces [f_l, f_r] using full PD control.

        Args:
            r : Reference vector   [phi_ref, theta_ref, psi_ref]
            y : Measurement vector [phi, theta, psi]  (angles only, no rates)

        Returns:
            u : np.ndarray [f_l, f_r]  motor forces (N)
        """
        # ── Unpack ────────────────────────────────────────────────────
        phi_ref, theta_ref, psi_ref = r[0], r[1], r[2]
        phi, theta, psi             = y[0], y[1], y[2]

        # ══════════════════════════════════════════════════════════════
        # LONGITUDINAL — pitch PD
        # ══════════════════════════════════════════════════════════════
        error_theta   = theta_ref - theta
        thetadot_filt = self._filtered_derivative_theta(theta)
        #thetadot_filt = y[4]

        F_fl    = (P.m1 * P.ell_1 + P.m2 * P.ell_2) * P.g / P.ell_T * np.cos(theta)
        F_tilde = self.kp_theta * error_theta - self.kd_theta * thetadot_filt
        F_cmd   = F_fl + F_tilde

        # ══════════════════════════════════════════════════════════════
        # LATERAL — successive loop closure
        # ══════════════════════════════════════════════════════════════

        # ── Outer loop: yaw PD → generates phi reference ──────────────
        error_psi   = psi_ref - psi
        psidot_filt = self._filtered_derivative_psi(psi)
        #psidot_filt = y[5]

        phi_ref_cmd = self.kp_psi * error_psi - self.kd_psi * psidot_filt

        # ── Inner loop: roll PD → generates torque command ────────────
        error_phi   = phi_ref_cmd - phi
        phidot_filt = self._filtered_derivative_phi(phi)
        #phidot_filt = y[3]

        tau_cmd = self.kp_phi * error_phi - self.kd_phi * phidot_filt

        # ══════════════════════════════════════════════════════════════
        # Mix [F, tau] → [f_l, f_r]
        # ══════════════════════════════════════════════════════════════
        # Mix [F, tau] → [f_l, f_r]
        u = P.mixer @ np.array([F_cmd, tau_cmd])
        u = np.clip(u, 0.0, np.inf)

        # No observer yet — return y (angles) padded with filtered rates as xhat
        xhat = np.array([phi, theta, psi, self._phidot_prev, self._thetadot_prev, self._psidot_prev])

        return u , xhat

    # ──────────────────────────────────────────────────────────────────
    # Tustin (bilinear) derivative filters — one per angle
    # ──────────────────────────────────────────────────────────────────
    def _filtered_derivative_theta(self, theta):
        """Low-pass filtered derivative of theta."""
        # thetadot = (self._b0_theta * theta
        #             + self._b1_theta * self._theta_prev
        #             - self._a1_theta * self._thetadot_prev)
        # self._theta_prev    = theta
        # self._thetadot_prev = thetadot
        ######################
        theta_diff = (theta - self._theta_prev) / P.ts
        self.thetadot_hat = self.b_theta * self.thetadot_hat + (1 - self.b_theta) * theta_diff
        self.theta_prev = theta
        return self.thetadot_hat

    def _filtered_derivative_phi(self, phi):
        """Low-pass filtered derivative of phi."""
        # phidot = (self._b0_phi * phi
        #           + self._b1_phi * self._phi_prev
        #           - self._a1_phi * self._phidot_prev)
        # self._phi_prev    = phi
        # self._phidot_prev = phidot
        ###################
        phi_diff = (phi - self._phi_prev) / P.ts
        self.phidot_hat = self.b_phi * self.phidot_hat + (1 - self.b_phi) * phi_diff
        self.phi_prev = phi
        return self.phidot_hat

    def _filtered_derivative_psi(self, psi):
        """Low-pass filtered derivative of psi."""
        # psidot = (self._b0_psi * psi
        #           + self._b1_psi * self._psi_prev
        #           - self._a1_psi * self._psidot_prev)
        # self._psi_prev    = psi
        # self._psidot_prev = psidot
        #####################
        psi_diff = (psi - self._psi_prev) / P.ts
        self.psidot_hat = self.b_psi * self.psidot_hat + (1 - self.b_psi) * psi_diff
        self.psi_prev = psi
        return self.psidot_hat