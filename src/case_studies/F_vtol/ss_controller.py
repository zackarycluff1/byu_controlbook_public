# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class VTOLControllerSS(ControllerBase):
    def __init__(self):
        # tuning parameters
        tr_theta = 0.4
        zeta = 0.9

        tr_h = 2.0

        M = 10  # time separation factor between inner and outer loop
        tr_z = tr_theta * M

        # check controllability
        if np.linalg.matrix_rank(cnt.ctrb(P.A_lat, P.B_lat)) != 4:
            raise ValueError("System not controllable")

        # compute lateral gains
        wn_theta = 0.5 * np.pi / (tr_theta * np.sqrt(1 - zeta**2))
        theta_char_poly = [1, 2 * zeta * wn_theta, wn_theta**2]
        theta_poles = np.roots(theta_char_poly)

        wn_z = 0.5 * np.pi / (tr_z * np.sqrt(1 - zeta**2))
        z_char_poly = [1, 2 * zeta * wn_z, wn_z**2]
        z_poles = np.roots(z_char_poly)

        des_poles_lat = np.hstack([theta_poles, z_poles])

        self.K = cnt.place(P.A_lat, P.B_lat, des_poles_lat)
        self.kr = (-1.0 / (P.Cr @ np.linalg.inv(P.A_lat - P.B_lat @ self.K) @ P.B_lat)).item()
        print("des_poles_lat:", des_poles_lat)
        print("K:", self.K)
        print("kr:", self.kr)

        # compute longitudinal gains
        wn_h = 0.5 * np.pi / (tr_h * np.sqrt(1 - zeta**2))
        h_char_poly = [1, 2 * zeta * wn_h, wn_h**2]
        h_poles = np.roots(h_char_poly)

        self.K_lon = cnt.place(P.A_lon, P.B_lon, h_poles)
        self.kr_lon = (-1.0 / (P.Cm_lon @ np.linalg.inv(P.A_lon - P.B_lon @ self.K_lon) @ P.B_lon)).item()
        print("h_poles:", h_poles)
        print("K_lon:", self.K_lon)
        print("kr_lon:", self.kr_lon)

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

    def update_with_state(self, r, x):
        # longitudinal (altitude) control
        x_lon = x[[1, 4]]
        r_lon = r[[1]]
        x_lon_tilde = x_lon - self.x_eq_lon
        r_lon_tilde = r_lon - self.r_eq_lon
        F_tilde = -self.K_lon @ x_lon_tilde + self.kr_lon * r_lon_tilde

        # lateral (side-to-side) control
        x_lat = x[[0, 2, 3, 5]]
        r_lat = r[[0]]
        x_lat_tilde = x_lat - self.x_eq_lat
        r_lat_tilde = r_lat - self.r_eq_lat
        tau_tilde = -self.K @ x_lat_tilde + self.kr * r_lat_tilde

        # combine in [F, tau] space and add equilibrium
        u_FT_tilde = np.hstack([F_tilde, tau_tilde])
        u = u_FT_tilde + self.u_eq

        u = self.saturate(u, u_max=P.force_max)
        return u