# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class VTOLControllerSSI(ControllerBase):
    def __init__(self, separate_integrator=True):
        # tuning parameters
        tr_theta = 0.4
        zeta = 0.9

        tr_h = 2.0

        M = 10  # time separation factor between inner and outer loop
        tr_z = tr_theta * M

        integrator_pole_lat = [-5.0]
        integrator_pole_lon = [-5.0]

        # augmented lateral system
        A1_lat = np.block([[P.A_lat, np.zeros((4, 1))], [-P.Cr, np.zeros(1)]])
        B1_lat = np.vstack((P.B_lat, 0))

        # check controllability
        if np.linalg.matrix_rank(cnt.ctrb(A1_lat, B1_lat)) != 5:
            raise ValueError("Lateral system not controllable")

        # compute lateral gains
        wn_theta = 0.5 * np.pi / (tr_theta * np.sqrt(1 - zeta**2))
        theta_char_poly = [1, 2 * zeta * wn_theta, wn_theta**2]
        theta_poles = np.roots(theta_char_poly)

        wn_z = 0.5 * np.pi / (tr_z * np.sqrt(1 - zeta**2))
        z_char_poly = [1, 2 * zeta * wn_z, wn_z**2]
        z_poles = np.roots(z_char_poly)

        des_sys_poles_lat = np.hstack([theta_poles, z_poles])
        des_poles_lat = np.hstack((des_sys_poles_lat, integrator_pole_lat))

        self.K1 = cnt.place(A1_lat, B1_lat, des_poles_lat)
        self.K = self.K1[:, :4]
        self.ki = self.K1[:, 4:]
        print("des_poles_lat:", des_poles_lat)
        print("K1:", self.K1)
        print("K:", self.K)
        print("ki:", self.ki)

        # augmented longitudinal system
        A1_lon = np.block([[P.A_lon, np.zeros((2, 1))], [-P.Cm_lon, np.zeros(1)]])
        B1_lon = np.vstack((P.B_lon, 0))

        # check controllability
        if np.linalg.matrix_rank(cnt.ctrb(A1_lon, B1_lon)) != 3:
            raise ValueError("Longitudinal system not controllable")

        # compute longitudinal gains
        wn_h = 0.5 * np.pi / (tr_h * np.sqrt(1 - zeta**2))
        h_char_poly = [1, 2 * zeta * wn_h, wn_h**2]
        h_sys_poles = np.roots(h_char_poly)
        des_poles_lon = np.hstack((h_sys_poles, integrator_pole_lon))

        self.K1_lon = cnt.place(A1_lon, B1_lon, des_poles_lon)
        self.K_lon = self.K1_lon[:, :2]
        self.ki_lon = self.K1_lon[:, 2:]
        print("des_poles_lon:", des_poles_lon)
        print("K1_lon:", self.K1_lon)
        print("K_lon:", self.K_lon)
        print("ki_lon:", self.ki_lon)

        # linearization point
        self.x_eq_lon = P.x_eq_lon
        self.x_eq_lat = P.x_eq_lat
        self.r_eq_lon = P.Cm_lon @ self.x_eq_lon
        self.r_eq_lat = P.Cr @ self.x_eq_lat
        self.u_eq = P.u_eq

        # dirty derivative variables
        sigma = 0.05  # cutoff freq for dirty derivative
        self.beta = (2 * sigma - P.ts) / (2 * sigma + P.ts)
        self.thetadot_hat = P.thetadot0
        self.theta_prev = P.theta0
        self.zdot_hat = P.zdot0
        self.z_prev = P.z0
        self.hdot_hat = P.hdot0
        self.h_prev = P.h0

        # integrator variables
        self.error_lat_prev = 0.0
        self.error_lat_integral = 0.0
        self.error_lon_prev = 0.0
        self.error_lon_integral = 0.0
        self.separate_integrator = separate_integrator

    def update_with_state(self, r, x):
        # longitudinal (altitude) control
        x_lon = x[[1, 4]]
        r_lon = r[[1]]
        x_lon_tilde = x_lon - self.x_eq_lon

        error_lon = r_lon - P.Cm_lon @ x_lon
        self.error_lon_integral += P.ts * (error_lon + self.error_lon_prev) / 2
        self.error_lon_prev = error_lon

        if self.separate_integrator:
            F_tilde = -self.K_lon @ x_lon_tilde - self.ki_lon @ self.error_lon_integral
        else:
            x1_lon_tilde = np.hstack((x_lon_tilde, self.error_lon_integral))
            F_tilde = -self.K1_lon @ x1_lon_tilde

        # lateral (side-to-side) control
        x_lat = x[[0, 2, 3, 5]]
        r_lat = r[[0]]
        x_lat_tilde = x_lat - self.x_eq_lat

        error_lat = r_lat - P.Cr @ x_lat
        self.error_lat_integral += P.ts * (error_lat + self.error_lat_prev) / 2
        self.error_lat_prev = error_lat

        if self.separate_integrator:
            tau_tilde = -self.K @ x_lat_tilde - self.ki @ self.error_lat_integral
        else:
            x1_lat_tilde = np.hstack((x_lat_tilde, self.error_lat_integral))
            tau_tilde = -self.K1 @ x1_lat_tilde

        # combine in [F, tau] space and add equilibrium
        u_FT_tilde = np.hstack([F_tilde, tau_tilde])
        u = u_FT_tilde + self.u_eq

        u = self.saturate(u, u_max=P.force_max)
        return u