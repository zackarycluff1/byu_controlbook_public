# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class VTOLControllerSSIDO(ControllerBase):
    """
    LQR + disturbance-observer controller for the VTOL system.

    Longitudinal subsystem: states [h, hdot], input F, output h  (Cm_lon)
    Lateral subsystem:      states [z, theta, zdot, thetadot], input tau, output z (Cm_lat_obs / Cr)

    Each subsystem mirrors the satellite SSI-DO pattern:
      - Augmented integrator state for steady-state error rejection
      - Disturbance-augmented Luenberger observer (RK4 integration)
      - Disturbance estimate subtracted from control output
    """

    def __init__(self, separate_integrator=False):
        self.separate_integrator = separate_integrator

        # ------------------------------------------------------------------ #
        #  LONGITUDINAL LQR DESIGN
        #  States: [h, hdot],  augmented with integrator -> 3 states
        #  P.A_lon: 2x2,  P.B_lon: 2x1,  P.Cm_lon: 1x2  (measures h only)
        # ------------------------------------------------------------------ #
        Q_lon = np.diag([20.0, 5.0, 50.0])   # [h, hdot, integrator]
        R_lon = np.array([[0.1]])

        A1_lon = np.block([
            [P.A_lon,               np.zeros((2, 1))],
            [-P.Cm_lon,             np.zeros((1, 1))],
        ])  # 3x3
        B1_lon = np.vstack((P.B_lon, np.zeros((1, 1))))  # 3x1

        if np.linalg.matrix_rank(cnt.ctrb(A1_lon, B1_lon)) != 3:
            raise ValueError("Longitudinal augmented system not controllable")

        K1_lon, *_ = cnt.lqr(A1_lon, B1_lon, Q_lon, R_lon)  # 1x3
        self.K_lon  = K1_lon[:, :2]   # 1x2  state feedback gains [h, hdot]
        self.ki_lon = K1_lon[:, 2:]   # 1x1  integrator gain

        if separate_integrator:
            print("K_lon:", self.K_lon)
            print("ki_lon:", self.ki_lon)
        else:
            print("K1_lon:", K1_lon)

        # Longitudinal equilibrium
        self.x_eq_lon = P.x_eq_lon.copy()                      # [0, 0]
        self.r_eq_lon = P.Cm_lon @ self.x_eq_lon        # 0.0
        self.u_eq_lon = np.array([P.u_eq[0]])                  # [F_eq]  ~14.7 N

        # Longitudinal integrator state
        self.lon_error_prev     = 0.0
        self.lon_error_integral = 0.0

        # ------------------------------------------------------------------ #
        #  LATERAL LQR DESIGN
        #  States: [z, theta, zdot, thetadot],  augmented with integrator -> 5 states
        #  P.A_lat: 4x4,  P.B_lat: 4x1,  P.Cr: 1x4  (commands z)
        # ------------------------------------------------------------------ #
        Q_lat = np.diag([20.0, 20.0, 5.0, 5.0, 50.0])   # [z, theta, zdot, thetadot, integrator]
        R_lat = np.array([[0.1]])

        A1_lat = np.block([
            [P.A_lat,   np.zeros((4, 1))],
            [-P.Cr,     np.zeros((1, 1))],
        ])  # 5x5
        B1_lat = np.vstack((P.B_lat, np.zeros((1, 1))))  # 5x1

        if np.linalg.matrix_rank(cnt.ctrb(A1_lat, B1_lat)) != 5:
            raise ValueError("Lateral augmented system not controllable")

        K1_lat, *_ = cnt.lqr(A1_lat, B1_lat, Q_lat, R_lat)  # 1x5
        self.K_lat  = K1_lat[:, :4]   # 1x4  state feedback gains [z, theta, zdot, thetadot]
        self.ki_lat = K1_lat[:, 4:]   # 1x1  integrator gain

        if separate_integrator:
            print("K_lat:", self.K_lat)
            print("ki_lat:", self.ki_lat)
        else:
            print("K1_lat:", K1_lat)

        # Lateral equilibrium
        self.x_eq_lat = P.x_eq_lat.copy()                      # [0, 0, 0, 0]
        self.r_eq_lat = P.Cr @ self.x_eq_lat            # 0.0
        self.u_eq_lat = np.array([P.u_eq[1]])                  # [0.0]

        # Lateral integrator state
        self.lat_error_prev     = 0.0
        self.lat_error_integral = 0.0

        # ------------------------------------------------------------------ #
        #  LONGITUDINAL OBSERVER DESIGN
        #  Augment with disturbance: [h, hdot, d] -> 3 states
        #  C2_lon: 1x3  (measures h only, via Cm_lon)
        # ------------------------------------------------------------------ #
        tr_h_obs      = 0.2
        zeta_h_obs    = 0.9
        lon_dist_pole = [-1.0]

        self.A2_lon = np.block([
            [P.A_lon,           P.B_lon          ],   # 2x3
            [np.zeros((1, 3))                    ],   # 1x3
        ])  # 3x3
        self.B2_lon = np.vstack((P.B_lon, np.zeros((1, 1))))    # 3x1
        self.C2_lon = np.block([P.Cm_lon, np.zeros((1, 1))])    # 1x3  (h only)

        if np.linalg.matrix_rank(cnt.ctrb(self.A2_lon.T, self.C2_lon.T)) != 3:
            raise ValueError("Longitudinal augmented system not observable")

        wn_h_obs      = 2.2 / tr_h_obs
        h_obs_poles   = np.roots([1, 2*zeta_h_obs*wn_h_obs, wn_h_obs**2])
        lon_obs_poles = np.hstack((h_obs_poles, lon_dist_pole))

        self.L2_lon = cnt.place(self.A2_lon.T, self.C2_lon.T, lon_obs_poles).T  # 3x1

        # Longitudinal observer state (tilde = perturbation from equilibrium)
        self.x2hat_tilde_lon = np.zeros(3)
        self.u_lon_prev      = np.zeros(1)
        self.x2_eq_lon       = np.hstack((self.x_eq_lon, [0.0]))   # [h_eq, hdot_eq, 0]

        # ------------------------------------------------------------------ #
        #  LATERAL OBSERVER DESIGN
        #  Augment with disturbance: [z, theta, zdot, thetadot, d] -> 5 states
        #
        #  CRITICAL: Use Cm_lat_obs (1x4, z only) NOT Cm_lat (2x4).
        #            Cm_lat_obs matches what the sensor actually provides for lateral.
        # ------------------------------------------------------------------ #
        tr_z_obs       = 0.6
        zeta_z_obs     = 0.9
        tr_theta_obs   = 0.2
        zeta_theta_obs = 0.9
        lat_dist_pole  = [-1.0]

        self.A2_lat = np.block([
            [P.A_lat,           P.B_lat          ],   # 4x5
            [np.zeros((1, 5))                    ],   # 1x5
        ])  # 5x5
        self.B2_lat = np.vstack((P.B_lat, np.zeros((1, 1))))           # 5x1
        # Cm_lat_obs is 1x4 (z only) — correct sensor model
        self.C2_lat = np.block([P.Cm_lat_obs, np.zeros((1, 1))])       # 1x5

        if np.linalg.matrix_rank(cnt.ctrb(self.A2_lat.T, self.C2_lat.T)) != 5:
            raise ValueError("Lateral augmented system not observable")

        wn_theta_obs    = 2.2 / tr_theta_obs
        theta_obs_poles = np.roots([1, 2*zeta_theta_obs*wn_theta_obs, wn_theta_obs**2])
        wn_z_obs        = 2.2 / tr_z_obs
        z_obs_poles     = np.roots([1, 2*zeta_z_obs*wn_z_obs, wn_z_obs**2])
        lat_obs_poles   = np.hstack((theta_obs_poles, z_obs_poles, lat_dist_pole))

        self.L2_lat = cnt.place(self.A2_lat.T, self.C2_lat.T, lat_obs_poles).T  # 5x1

        # Lateral observer state (tilde = perturbation from equilibrium)
        self.x2hat_tilde_lat = np.zeros(5)
        self.u_lat_prev      = np.zeros(1)
        self.x2_eq_lat       = np.hstack((self.x_eq_lat, [0.0]))   # [z_eq, theta_eq, zdot_eq, thetadot_eq, 0]

    # ---------------------------------------------------------------------- #
    #  UPDATE WITH FULL STATE  (no observer — for debugging/testing)
    # ---------------------------------------------------------------------- #
    def update_with_state(self, r, x):
        """
        r : [z_ref, h_ref]
        x : full state [z, theta, zdot, thetadot, h, hdot]
        """
        r_lat = float(r[0])
        r_lon = float(r[1])

        x_lat = np.array(x[:4], dtype=float)   # [z, theta, zdot, thetadot]
        x_lon = np.array(x[4:], dtype=float)   # [h, hdot]

        # ---------- longitudinal ----------
        x_tilde_lon = x_lon - self.x_eq_lon

        lon_error = r_lon - float(P.Cm_lon @ x_lon)
        self.lon_error_integral += P.ts * (lon_error + self.lon_error_prev) / 2
        self.lon_error_prev = lon_error

        if self.separate_integrator:
            u_tilde_lon = float(-(self.K_lon @ x_tilde_lon)) \
                          - float(self.ki_lon * self.lon_error_integral)
        else:
            x1_tilde_lon = np.hstack((x_tilde_lon, self.lon_error_integral))
            u_tilde_lon  = float(-np.hstack((self.K_lon, self.ki_lon)) @ x1_tilde_lon)

        F_unsat = u_tilde_lon + self.u_eq_lon[0]
        F = float(self.saturate(np.array([F_unsat]), u_max=P.force_max))
        self.u_lon_prev = np.array([F])

        # ---------- lateral ----------
        x_tilde_lat = x_lat - self.x_eq_lat

        lat_error = r_lat - float(P.Cr @ x_lat)
        self.lat_error_integral += P.ts * (lat_error + self.lat_error_prev) / 2
        self.lat_error_prev = lat_error

        if self.separate_integrator:
            u_tilde_lat = float(-(self.K_lat @ x_tilde_lat)) \
                          - float(self.ki_lat * self.lat_error_integral)
        else:
            x1_tilde_lat = np.hstack((x_tilde_lat, self.lat_error_integral))
            u_tilde_lat  = float(-np.hstack((self.K_lat, self.ki_lat)) @ x1_tilde_lat)

        tau_unsat = u_tilde_lat + self.u_eq_lat[0]
        tau = float(self.saturate(np.array([tau_unsat]), u_max=P.force_max))
        self.u_lat_prev = np.array([tau])

        return np.array([F, tau])

    # ---------------------------------------------------------------------- #
    #  UPDATE WITH MEASUREMENT  (full observer-based control)
    # ---------------------------------------------------------------------- #
    def update_with_measurement(self, r, y):
        """
        r : [z_ref, h_ref]
        y : [z, h]  — matches Dynamics.h() returning [z, h]
              y[0] = z  -> lateral observer  (Cm_lat_obs: z only)
              y[1] = h  -> longitudinal observer (Cm_lon: h only)
        """
        r_lat = float(r[0])
        r_lon = float(r[1])

        y_lat = np.asarray([y[0]], dtype=float)   # [z]
        y_lon = np.asarray([y[1]], dtype=float)   # [h]

        # ---------- longitudinal observer + control ----------
        xhat_lon, dhat_lon = self._lon_observer_rk4_step(y_lon)
        x_tilde_lon = xhat_lon - self.x_eq_lon

        lon_error = r_lon - P.Cm_lon @ xhat_lon
        self.lon_error_integral += P.ts * (lon_error + self.lon_error_prev) / 2
        self.lon_error_prev = lon_error

        if self.separate_integrator:
            u_tilde_lon = -(self.K_lon @ x_tilde_lon) \
                          - self.ki_lon * self.lon_error_integral
        else:
            x1_tilde_lon = np.hstack((x_tilde_lon, self.lon_error_integral))
            u_tilde_lon = -np.hstack((self.K_lon, self.ki_lon)) @ x1_tilde_lon

        F_unsat = u_tilde_lon + self.u_eq_lon[0] - float(dhat_lon)
        F = float(self.saturate(np.array([F_unsat]), u_max=P.force_max))
        self.u_lon_prev = np.array([F])

        # ---------- lateral observer + control ----------
        xhat_lat, dhat_lat = self._lat_observer_rk4_step(y_lat)
        x_tilde_lat = xhat_lat - self.x_eq_lat

        lat_error = r_lat - P.Cr @ xhat_lat
        self.lat_error_integral += P.ts * (lat_error + self.lat_error_prev) / 2
        self.lat_error_prev = lat_error

        if self.separate_integrator:
            u_tilde_lat = -(self.K_lat @ x_tilde_lat) \
                          - self.ki_lat * self.lat_error_integral
        else:
            x1_tilde_lat = np.hstack((x_tilde_lat, self.lat_error_integral))
            u_tilde_lat = -np.hstack((self.K_lat, self.ki_lat)) @ x1_tilde_lat

        tau_unsat = u_tilde_lat + self.u_eq_lat[0] - dhat_lat
        tau = float(self.saturate(np.array([tau_unsat]), u_max=P.force_max))
        self.u_lat_prev = np.array([tau])

        u    = np.array([F, tau])
        xhat = np.hstack((xhat_lat, xhat_lon))             # [z, theta, zdot, thetadot, h, hdot]
        dhat = np.array([dhat_lat, dhat_lon])
        return u, xhat, dhat

    # ---------------------------------------------------------------------- #
    #  LONGITUDINAL OBSERVER  (disturbance-augmented, RK4)
    # ---------------------------------------------------------------------- #
    def _lon_observer_f(self, x2hat_tilde, y):
        x2hat     = x2hat_tilde + self.x2_eq_lon
        y_error   = y - self.C2_lon @ x2hat
        u_tilde   = self.u_lon_prev - self.u_eq_lon
        x2hat_dot = (self.A2_lon @ x2hat_tilde
                     + self.B2_lon @ u_tilde
                     + self.L2_lon @ y_error)
        return x2hat_dot

    def _lon_observer_rk4_step(self, y):
        k1 = self._lon_observer_f(self.x2hat_tilde_lon, y)
        k2 = self._lon_observer_f(self.x2hat_tilde_lon + P.ts/2 * k1, y)
        k3 = self._lon_observer_f(self.x2hat_tilde_lon + P.ts/2 * k2, y)
        k4 = self._lon_observer_f(self.x2hat_tilde_lon + P.ts   * k3, y)
        self.x2hat_tilde_lon += P.ts * (k1 + 2*k2 + 2*k3 + k4) / 6

        xhat_lon = self.x2hat_tilde_lon[:2] + self.x_eq_lon   # [h, hdot]
        dhat_lon = self.x2hat_tilde_lon[2:3].copy()            # [d_lon]
        return xhat_lon, dhat_lon

    # ---------------------------------------------------------------------- #
    #  LATERAL OBSERVER  (disturbance-augmented, RK4)
    # ---------------------------------------------------------------------- #
    def _lat_observer_f(self, x2hat_tilde, y):
        x2hat     = x2hat_tilde + self.x2_eq_lat
        y_error   = y - self.C2_lat @ x2hat               # z only
        u_tilde   = self.u_lat_prev - self.u_eq_lat
        x2hat_dot = (self.A2_lat @ x2hat_tilde
                     + self.B2_lat @ u_tilde
                     + self.L2_lat @ y_error)
        return x2hat_dot

    def _lat_observer_rk4_step(self, y):
        k1 = self._lat_observer_f(self.x2hat_tilde_lat, y)
        k2 = self._lat_observer_f(self.x2hat_tilde_lat + P.ts/2 * k1, y)
        k3 = self._lat_observer_f(self.x2hat_tilde_lat + P.ts/2 * k2, y)
        k4 = self._lat_observer_f(self.x2hat_tilde_lat + P.ts   * k3, y)
        self.x2hat_tilde_lat += P.ts * (k1 + 2*k2 + 2*k3 + k4) / 6

        xhat_lat = self.x2hat_tilde_lat[:4] + self.x_eq_lat   # [z, theta, zdot, thetadot]
        dhat_lat = self.x2hat_tilde_lat[4:5].copy()            # [d_lat]
        return xhat_lat, dhat_lat