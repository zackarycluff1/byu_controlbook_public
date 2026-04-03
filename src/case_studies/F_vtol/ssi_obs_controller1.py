# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class VTOLControllerSSIO(ControllerBase):
    def __init__(self, separate_integrator=False):
        # ------------------------------------------------------------------ #
        #  LONGITUDINAL DESIGN  — matches ssi_controller poles exactly
        # ------------------------------------------------------------------ #
        tr_h   = 2.0
        zeta_h = 0.9
        lon_integrator_pole = [-5.0]

        A1_lon = np.block([[P.A_lon, np.zeros((2, 1))],
                           [-P.Cm_lon, np.zeros((1, 1))]])
        B1_lon = np.vstack((P.B_lon, np.zeros((1, 1))))

        if np.linalg.matrix_rank(cnt.ctrb(A1_lon, B1_lon)) != 3:
            raise ValueError("Longitudinal system not controllable")

        # Use the same wn formula as ssi_controller (2nd-order rise time)
        wn_h        = 0.5 * np.pi / (tr_h * np.sqrt(1 - zeta_h**2))
        h_poles     = np.roots([1, 2 * zeta_h * wn_h, wn_h**2])
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
        #  LATERAL DESIGN  — matches ssi_controller poles exactly
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

        # Use the same wn formula as ssi_controller (2nd-order rise time)
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
        #  LONGITUDINAL OBSERVER
        # ------------------------------------------------------------------ #
        M_obs_lon  = 10.0
        tr_h_obs   = tr_h / M_obs_lon
        zeta_h_obs = 0.707

        if np.linalg.matrix_rank(cnt.ctrb(P.A_lon.T, P.Cm_lon.T)) != 2:
            raise ValueError("Longitudinal system not observable")

        wn_h_obs    = 0.5 * np.pi / (tr_h_obs * np.sqrt(1 - zeta_h_obs**2))
        h_obs_poles = np.roots([1, 2 * zeta_h_obs * wn_h_obs, wn_h_obs**2])

        self.L_lon = cnt.place(P.A_lon.T, P.Cm_lon.T, h_obs_poles).T
        print("L_lon^T:", self.L_lon.T)

        self.xhat_lon   = self.x_eq_lon.copy()
        self.u_lon_prev = np.zeros(1)

        # ------------------------------------------------------------------ #
        #  LATERAL OBSERVER
        # ------------------------------------------------------------------ #
        # With tr_theta=0.4, M_obs=10 gives tr_obs=0.04 which is extremely fast
        # and causes huge L gains with a single z measurement.
        # M_obs=3 still keeps observer faster than controller but stays well-conditioned.
        M_obs_lat      = 3.0
        tr_theta_obs   = tr_theta / M_obs_lat
        zeta_theta_obs = 0.9
        tr_z_obs       = tr_z / M_obs_lat
        zeta_z_obs     = 0.9

        if np.linalg.matrix_rank(cnt.ctrb(P.A_lat.T, P.Cm_lat_obs.T)) != 4:
            raise ValueError("Lateral system not observable")

        wn_theta_obs    = 0.5 * np.pi / (tr_theta_obs * np.sqrt(1 - zeta_theta_obs**2))
        theta_obs_poles = np.roots([1, 2 * zeta_theta_obs * wn_theta_obs, wn_theta_obs**2])
        wn_z_obs        = 0.5 * np.pi / (tr_z_obs * np.sqrt(1 - zeta_z_obs**2))
        z_obs_poles     = np.roots([1, 2 * zeta_z_obs * wn_z_obs, wn_z_obs**2])

        lat_obs_poles = np.hstack((theta_obs_poles, z_obs_poles))
        self.L_lat = cnt.place(P.A_lat.T, P.Cm_lat_obs.T, lat_obs_poles).T
        print("L_lat^T:", self.L_lat.T)

        self.xhat_lat   = self.x_eq_lat.copy()
        self.u_lat_prev = np.zeros(1)

    # ---------------------------------------------------------------------- #
    #  UPDATE STEP (state input — extracts y from full state for the observers)
    # ---------------------------------------------------------------------- #
    def update_with_state(self, r, x):
        """
        Called when controller_input="state".
        Full state vector layout (from ssi_controller.py):
            x[0]=z, x[1]=h, x[2]=theta, x[3]=zdot, x[4]=hdot, x[5]=thetadot

        Extracts y = [z, theta, h] and passes to update_with_measurement
        so the observers still run exactly as designed.
        """
        y = np.array([x[0], x[1]], dtype=float)  # [z, h] — matches Dynamics.h()
        return self.update_with_measurement(r, y)

    # ---------------------------------------------------------------------- #
    #  UPDATE STEP (measurement input)
    # ---------------------------------------------------------------------- #
    def update_with_measurement(self, r, y):
        """
        r : [z_ref, h_ref]
        y : [z, h]  — from Dynamics.h() which returns state[:2]
              y[0]=z  → lateral observer (Cm_lat measures z only)
              y[1]=h  → longitudinal observer (Cm_lon measures h)
        """
        r_lat = float(r[0])
        r_lon = float(r[1])

        y_lat = np.asarray(y[0:1], dtype=float).flatten()  # [z]
        y_lon = np.asarray(y[1:2], dtype=float).flatten()  # [h]

        # ---------- longitudinal ----------
        xhat_lon    = self._lon_observer_step(y_lon)
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

        F = float(np.clip(u_tilde_lon + self.u_eq_lon[0], -P.force_max, P.force_max))
        self.u_lon_prev = np.array([F])

        # ---------- lateral ----------
        xhat_lat    = self._lat_observer_step(y_lat)
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

        tau = float(np.clip(u_tilde_lat + self.u_eq_lat[0], -P.force_max, P.force_max))
        self.u_lat_prev = np.array([tau])

        u    = np.array([F, tau])
        xhat = np.hstack((xhat_lat, xhat_lon))  # [z, theta, zdot, thetadot, h, hdot]
        return u, xhat

    # ---------------------------------------------------------------------- #
    #  LONGITUDINAL OBSERVER  (closed-loop — needs h measurement)
    # ---------------------------------------------------------------------- #
    def _lon_observer_f(self, xhat, y):
        y_error  = y - (P.Cm_lon @ xhat)
        x_tilde  = xhat - self.x_eq_lon
        u_tilde  = self.u_lon_prev - self.u_eq_lon
        xhat_dot = P.A_lon @ x_tilde + P.B_lon @ u_tilde + self.L_lon @ y_error
        return xhat_dot

    def _lon_observer_step(self, y):
        k1 = self._lon_observer_f(self.xhat_lon, y)
        k2 = self._lon_observer_f(self.xhat_lon + P.ts / 2 * k1, y)
        k3 = self._lon_observer_f(self.xhat_lon + P.ts / 2 * k2, y)
        k4 = self._lon_observer_f(self.xhat_lon + P.ts * k3, y)
        self.xhat_lon += P.ts * (k1 + 2*k2 + 2*k3 + k4) / 6
        return self.xhat_lon

    # ---------------------------------------------------------------------- #
    #  LATERAL OBSERVER  (closed-loop — uses y_lat measurement)
    # ---------------------------------------------------------------------- #
    def _lat_observer_f(self, xhat, y):
        y_error  = y - (P.Cm_lat_obs @ xhat)
        x_tilde  = xhat - self.x_eq_lat
        u_tilde  = self.u_lat_prev - self.u_eq_lat
        xhat_dot = P.A_lat @ x_tilde + P.B_lat @ u_tilde + self.L_lat @ y_error
        return xhat_dot

    def _lat_observer_step(self, y):
        k1 = self._lat_observer_f(self.xhat_lat, y)
        k2 = self._lat_observer_f(self.xhat_lat + P.ts / 2 * k1, y)
        k3 = self._lat_observer_f(self.xhat_lat + P.ts / 2 * k2, y)
        k4 = self._lat_observer_f(self.xhat_lat + P.ts * k3, y)
        self.xhat_lat += P.ts * (k1 + 2*k2 + 2*k3 + k4) / 6
        return self.xhat_lat