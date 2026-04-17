# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class VTOLControllerSSIDO(ControllerBase):
    def __init__(self, separate_integrator=False):
        # ------------------------------------------------------------------ #
        #  LONGITUDINAL DESIGN  — unchanged from SSIO
        # ------------------------------------------------------------------ #
        tr_h   = 2.0
        zeta_h = 0.9
        lon_integrator_pole = [-5.0]

        A1_lon = np.block([[P.A_lon, np.zeros((2, 1))],
                           [-P.Cm_lon, np.zeros((1, 1))]])
        B1_lon = np.vstack((P.B_lon, np.zeros((1, 1))))

        if np.linalg.matrix_rank(cnt.ctrb(A1_lon, B1_lon)) != 3:
            raise ValueError("Longitudinal system not controllable")

        wn_h = 0.5 * np.pi / (tr_h * np.sqrt(1 - zeta_h**2))
        h_poles = np.roots([1, 2 * zeta_h * wn_h, wn_h**2])
        lon_des_poles = np.hstack([h_poles, lon_integrator_pole])

        K1_lon = cnt.place(A1_lon, B1_lon, lon_des_poles)
        self.K_lon  = K1_lon[:, :2]
        self.ki_lon = K1_lon[:, 2:]

        self.x_eq_lon = P.x_eq_lon.astype(float)
        self.u_eq_lon = np.array([P.u_eq[0]], dtype=float)

        self.lon_error_prev     = 0.0
        self.lon_error_integral = 0.0
        self.lon_separate_integrator = separate_integrator

        # ------------------------------------------------------------------ #
        #  LATERAL DESIGN  — unchanged from SSIO
        # ------------------------------------------------------------------ #
        tr_theta   = 0.4
        zeta_theta = 0.9
        M          = 10
        tr_z       = tr_theta * M
        zeta_z     = 0.9
        lat_integrator_pole = [-5.0]

        A1_lat = np.block([[P.A_lat, np.zeros((4, 1))],
                           [-P.Cr,   np.zeros((1, 1))]])
        B1_lat = np.vstack((P.B_lat, np.zeros((1, 1))))

        if np.linalg.matrix_rank(cnt.ctrb(A1_lat, B1_lat)) != 5:
            raise ValueError("Lateral system not controllable")

        wn_theta      = 0.5 * np.pi / (tr_theta * np.sqrt(1 - zeta_theta**2))
        theta_poles   = np.roots([1, 2 * zeta_theta * wn_theta, wn_theta**2])
        wn_z          = 0.5 * np.pi / (tr_z * np.sqrt(1 - zeta_z**2))
        z_poles       = np.roots([1, 2 * zeta_z * wn_z, wn_z**2])
        lat_des_poles = np.hstack([theta_poles, z_poles, lat_integrator_pole])

        K1_lat = cnt.place(A1_lat, B1_lat, lat_des_poles)
        self.K_lat  = K1_lat[:, :4]
        self.ki_lat = K1_lat[:, 4:]

        self.x_eq_lat = P.x_eq_lat.astype(float)
        self.u_eq_lat = np.array([P.u_eq[1]], dtype=float)

        self.lat_error_prev     = 0.0
        self.lat_error_integral = 0.0
        self.lat_separate_integrator = separate_integrator

        # ------------------------------------------------------------------ #
        #  LONGITUDINAL DISTURBANCE OBSERVER
        #  Augmented state: [h, hdot, d_lon]  (2 states + 1 disturbance)
        #  A2_lon is 3x3, B2_lon is 3x1, C2_lon is 1x3
        # ------------------------------------------------------------------ #
        M_obs_lon  = 10.0
        tr_h_obs   = tr_h / M_obs_lon
        zeta_h_obs = 0.707
        lon_disturbance_pole = [-1.0]  # tune: more negative = faster rejection

        self.A2_lon = np.block([[P.A_lon, P.B_lon],
                                [np.zeros((1, 3))]])
        self.B2_lon = np.block([[P.B_lon],
                                [np.zeros((1, 1))]])
        self.C2_lon = np.block([P.Cm_lon, np.zeros((1, 1))])

        if np.linalg.matrix_rank(cnt.obsv(self.A2_lon, self.C2_lon)) != 3:
            raise ValueError("Augmented longitudinal system not observable")

        wn_h_obs    = 0.5 * np.pi / (tr_h_obs * np.sqrt(1 - zeta_h_obs**2))
        h_obs_poles = np.roots([1, 2 * zeta_h_obs * wn_h_obs, wn_h_obs**2])
        lon_obs_poles = np.hstack((h_obs_poles, lon_disturbance_pole))

        self.L2_lon = cnt.place(self.A2_lon.T, self.C2_lon.T, lon_obs_poles).T
        print("L2_lon^T:", self.L2_lon.T)

        # augmented observer state: [h_tilde, hdot_tilde, dhat_lon]
        self.x2hat_tilde_lon = np.zeros(3)
        self.x2_eq_lon = np.hstack((self.x_eq_lon, [0.0]))
        self.u_lon_prev = np.zeros(1)

        # ------------------------------------------------------------------ #
        #  LATERAL DISTURBANCE OBSERVER
        #  Augmented state: [z, theta, zdot, thetadot, d_lat]  (4 + 1)
        #  A2_lat is 5x5, B2_lat is 5x1, C2_lat is 1x5
        # ------------------------------------------------------------------ #
        M_obs_lat      = 3.0
        tr_theta_obs   = tr_theta / M_obs_lat
        zeta_theta_obs = 0.9
        tr_z_obs       = tr_z / M_obs_lat
        zeta_z_obs     = 0.9
        lat_disturbance_pole = [-1.0]  # tune: more negative = faster rejection

        self.A2_lat = np.block([[P.A_lat, P.B_lat],
                                [np.zeros((1, 5))]])
        self.B2_lat = np.block([[P.B_lat],
                                [np.zeros((1, 1))]])
        self.C2_lat = np.block([P.Cm_lat_obs, np.zeros((1, 1))])

        if np.linalg.matrix_rank(cnt.obsv(self.A2_lat, self.C2_lat)) != 5:
            raise ValueError("Augmented lateral system not observable")

        wn_theta_obs    = 0.5 * np.pi / (tr_theta_obs * np.sqrt(1 - zeta_theta_obs**2))
        theta_obs_poles = np.roots([1, 2 * zeta_theta_obs * wn_theta_obs, wn_theta_obs**2])
        wn_z_obs        = 0.5 * np.pi / (tr_z_obs * np.sqrt(1 - zeta_z_obs**2))
        z_obs_poles     = np.roots([1, 2 * zeta_z_obs * wn_z_obs, wn_z_obs**2])
        lat_obs_poles   = np.hstack((theta_obs_poles, z_obs_poles, lat_disturbance_pole))

        self.L2_lat = cnt.place(self.A2_lat.T, self.C2_lat.T, lat_obs_poles).T
        print("L2_lat^T:", self.L2_lat.T)

        # augmented observer state: [z_tilde, theta_tilde, zdot_tilde, thetadot_tilde, dhat_lat]
        self.x2hat_tilde_lat = np.zeros(5)
        self.x2_eq_lat = np.hstack((self.x_eq_lat, [0.0]))
        self.u_lat_prev = np.zeros(1)

    # ---------------------------------------------------------------------- #
    #  UPDATE STEP (state input)
    # ---------------------------------------------------------------------- #
    def update_with_state(self, r, x):
        y = np.array([x[0], x[1]], dtype=float)  # [z, h]
        return self.update_with_measurement(r, y)

    # ---------------------------------------------------------------------- #
    #  UPDATE STEP (measurement input)
    # ---------------------------------------------------------------------- #
    def update_with_measurement(self, r, y):
        """
        r : [z_ref, h_ref]
        y : [z, h]
        """
        r_lat = float(r[0])
        r_lon = float(r[1])

        y_lat = np.asarray(y[0:1], dtype=float).flatten()  # [z]
        y_lon = np.asarray(y[1:2], dtype=float).flatten()  # [h]

        # ---------- longitudinal ----------
        xhat_lon, dhat_lon = self._lon_observer_step(y_lon)
        x_tilde_lon = xhat_lon - self.x_eq_lon

        lon_error = r_lon - float((P.Cm_lon @ xhat_lon).flat[0])
        self.lon_error_integral += P.ts * (lon_error + self.lon_error_prev) / 2
        self.lon_error_prev = lon_error

        if self.lon_separate_integrator:
            u_tilde_lon = float((-(self.K_lon @ x_tilde_lon)).flat[0]) \
                          - float((self.ki_lon * self.lon_error_integral).flat[0])
        else:
            x1_tilde_lon = np.hstack((x_tilde_lon, [self.lon_error_integral]))
            u_tilde_lon  = float((-(np.hstack((self.K_lon, self.ki_lon)) @ x1_tilde_lon)).flat[0])

        # subtract disturbance estimate from control (key disturbance observer step)
        u_tilde_lon -= float(dhat_lon)

        F = float(np.clip(u_tilde_lon + self.u_eq_lon[0], -P.force_max, P.force_max))
        self.u_lon_prev = np.array([F])

        # ---------- lateral ----------
        xhat_lat, dhat_lat = self._lat_observer_step(y_lat)
        x_tilde_lat = xhat_lat - self.x_eq_lat

        lat_error = r_lat - float((P.Cr @ xhat_lat).flat[0])
        self.lat_error_integral += P.ts * (lat_error + self.lat_error_prev) / 2
        self.lat_error_prev = lat_error

        if self.lat_separate_integrator:
            u_tilde_lat = float((-(self.K_lat @ x_tilde_lat)).flat[0]) \
                          - float((self.ki_lat * self.lat_error_integral).flat[0])
        else:
            x1_tilde_lat = np.hstack((x_tilde_lat, [self.lat_error_integral]))
            u_tilde_lat  = float((-(np.hstack((self.K_lat, self.ki_lat)) @ x1_tilde_lat)).flat[0])

        # subtract disturbance estimate from control (key disturbance observer step)
        u_tilde_lat -= float(dhat_lat)

        tau = float(np.clip(u_tilde_lat + self.u_eq_lat[0], -P.force_max, P.force_max))
        self.u_lat_prev = np.array([tau])

        u    = np.array([F, tau])
        xhat = np.hstack((xhat_lat, xhat_lon))
        dhat = np.array([float(dhat_lon), float(dhat_lat)])
        return u, xhat, dhat

    # ---------------------------------------------------------------------- #
    #  LONGITUDINAL DISTURBANCE OBSERVER
    # ---------------------------------------------------------------------- #
    def _lon_observer_f(self, x2hat_tilde, y):
        x2hat   = x2hat_tilde + self.x2_eq_lon
        y_error = y - (self.C2_lon @ x2hat)
        u_tilde = self.u_lon_prev - self.u_eq_lon
        x2hat_dot = (self.A2_lon @ x2hat_tilde
                     + self.B2_lon @ u_tilde
                     + self.L2_lon @ y_error)
        return x2hat_dot

    def _lon_observer_step(self, y):
        k1 = self._lon_observer_f(self.x2hat_tilde_lon, y)
        k2 = self._lon_observer_f(self.x2hat_tilde_lon + P.ts / 2 * k1, y)
        k3 = self._lon_observer_f(self.x2hat_tilde_lon + P.ts / 2 * k2, y)
        k4 = self._lon_observer_f(self.x2hat_tilde_lon + P.ts * k3, y)
        self.x2hat_tilde_lon += P.ts * (k1 + 2*k2 + 2*k3 + k4) / 6

        xhat_tilde = self.x2hat_tilde_lon[:-1]
        xhat = xhat_tilde + self.x_eq_lon
        dhat = self.x2hat_tilde_lon[-1]
        return xhat, dhat

    # ---------------------------------------------------------------------- #
    #  LATERAL DISTURBANCE OBSERVER
    # ---------------------------------------------------------------------- #
    def _lat_observer_f(self, x2hat_tilde, y):
        x2hat   = x2hat_tilde + self.x2_eq_lat
        y_error = y - (self.C2_lat @ x2hat)
        u_tilde = self.u_lat_prev - self.u_eq_lat
        x2hat_dot = (self.A2_lat @ x2hat_tilde
                     + self.B2_lat @ u_tilde
                     + self.L2_lat @ y_error)
        return x2hat_dot

    def _lat_observer_step(self, y):
        k1 = self._lat_observer_f(self.x2hat_tilde_lat, y)
        k2 = self._lat_observer_f(self.x2hat_tilde_lat + P.ts / 2 * k1, y)
        k3 = self._lat_observer_f(self.x2hat_tilde_lat + P.ts / 2 * k2, y)
        k4 = self._lat_observer_f(self.x2hat_tilde_lat + P.ts * k3, y)
        self.x2hat_tilde_lat += P.ts * (k1 + 2*k2 + 2*k3 + k4) / 6

        xhat_tilde = self.x2hat_tilde_lat[:-1]
        xhat = xhat_tilde + self.x_eq_lat
        dhat = self.x2hat_tilde_lat[-1]
        return xhat, dhat