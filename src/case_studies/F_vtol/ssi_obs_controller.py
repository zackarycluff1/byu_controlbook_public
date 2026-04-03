# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class VTOLControllerSSIO(ControllerBase):
    def __init__(self, separate_integrator=False):
        # longitudinal tuning parameters
        tr_h = 2.0
        zeta_h = 0.707
        integrator_pole = [-5.0]

        # lateral tuning parameters
        tr_theta = 2.0
        zeta_theta = 0.707

        M = 10  # time separation factor between inner and outer loop
        tr_z = tr_theta * M
        zeta_z = 0.9
        integrator_pole = [-1.0]

        # longitudinal augmented system
        A1 = np.block([[P.A_lon, np.zeros((2, 1))], [-P.Cm_lon, np.zeros(1)]])
        B1 = np.vstack((P.B_lon, 0))

        # lateral augmented system
        A1 = np.block([[P.A, np.zeros((4, 1))], [-P.Cr, np.zeros(1)]])
        B1 = np.vstack((P.B, 0))

        # check longitudinal controllability
        if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 3:
            raise ValueError("Longitudinal system not controllable")

        # check lateral controllability
        if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 5:
            raise ValueError("System not controllable")
        
        # compute longitudinal gains
        wn_h = 0.5 * np.pi / (tr_h * np.sqrt(1 - zeta_h**2))
        h_char_poly = [1, 2 * zeta_h * wn_h, wn_h**2]
        h_poles = np.roots(h_char_poly)

        # compute lateral gains
        wn_theta = 0.5 * np.pi / (tr_theta * np.sqrt(1 - zeta_theta**2))
        theta_char_poly = [1, 2 * zeta_theta * wn_theta, wn_theta**2]
        theta_poles = np.roots(theta_char_poly)

        wn_z = 0.5 * np.pi / (tr_z * np.sqrt(1 - zeta_z**2))
        z_char_poly = [1, 2 * zeta_z * wn_z, wn_z**2]
        z_poles = np.roots(z_char_poly)

        des_poles = np.hstack([theta_poles, z_poles, integrator_pole])

        self.K1 = cnt.place(A1, B1, des_poles)
        self.K = self.K1[:, :4]
        self.ki = self.K1[:, 4:]

        # longitudinal linearization point
        self.x_eq = P.x_eq_lon
        self.r_eq = P.Cm_lon @ self.x_eq
        self.x1_eq = np.hstack((self.x_eq, 0))

        # lateral linearization point
        self.x_eq = P.x_eq_lat
        self.r_eq = P.Cm_lat @ self.x_eq
        self.u_eq = P.u_eq

        # longitudinal integrator variables
        self.lon_error_prev = 0.0
        self.lon_error_integral = 0.0
        self.lon_separate_integrator = separate_integrator

        # lateral integrator variables
        self.lat_error_prev = 0.0
        self.lat_error_integral = 0.0
        self.lat_separate_integrator = separate_integrator

        ####### OBSERVER DESIGN ########
        # longitduinal observer tuning parameters
        M_obs_lon = 10.0  # time scale separation between controller and observer
        tr_h_obs = tr_h / M_obs_lon
        zeta_h_obs = 0.707

        # lateral observer tuning parameters
        M_obs_lat = 10.0  # time scale separation between controller and observer
        tr_theta_obs = tr_theta / M_obs_lat
        zeta_theta_obs = 0.9
        tr_z_obs = tr_z / M_obs_lat
        zeta_z_obs = 0.9

        # check longitudinal observability
        if np.linalg.matrix_rank(cnt.ctrb(P.A_lon.T, P.Cm_lon.T)) != 4:
            raise ValueError("System not observable")
        
        # check lateral observability
        if np.linalg.matrix_rank(cnt.ctrb(P.A_lat.T, P.Cm_lat.T)) != 4:
            raise ValueError("System not observable")

        # compute observer gain matrix

        # longitudinal observer gains
        wn_h_obs = 2.2 / tr_h_obs
        h_obs_char_poly = [1, 2 * zeta_h_obs * wn_h_obs, wn_h_obs**2]
        h_obs_poles = np.roots(h_obs_char_poly)

        # lateral observer gains
        wn_theta_obs = 2.2 / tr_theta_obs
        theta_obs_char_poly = [1, 2 * zeta_theta_obs * wn_theta_obs, wn_theta_obs**2]
        theta_obs_poles = np.roots(theta_obs_char_poly)

        wn_z_obs = 2.2 / tr_z_obs
        z_obs_char_poly = [1, 2 * zeta_z_obs * wn_z_obs, wn_z_obs**2]
        z_obs_poles = np.roots(z_obs_char_poly)

        obs_poles = np.hstack((z_obs_poles, h_obs_poles, theta_obs_poles))
        self.L = cnt.place(P.A_lat.T, P.Cm_lat.T, obs_poles).T
        print("L^T:", self.L.T)

        # observer variables
        self.xhat_tilde = np.zeros(4)
        self.u_prev = np.zeros(1)

    def update_with_measurement(self, r, y):
        # update the observer with the measurement
        xhat = self.observer_rk4_step(y)

        x_tilde = xhat - self.x_eq
        r_tilde = r - self.r_eq

        # integrate error
        error = r - P.Cr @ xhat  # can also use tilde vars (eq subtracts out)
        self.error_integral += P.ts * (error + self.error_prev) / 2
        self.error_prev = error

        # compute feedback control
        if self.separate_integrator:
            u_tilde = -self.K @ x_tilde - self.ki @ self.error_integral
        else:
            x1_tilde = np.hstack((x_tilde, self.error_integral))
            u_tilde = -self.K1 @ x1_tilde

        # convert back to original variables
        u_unsat = u_tilde + self.u_eq
        u = self.saturate(u_unsat, u_max=P.torque_max)

        # save the previous control input for the observer
        self.u_prev = u

        return u, xhat

    def observer_f(self, xhat, y):
        y_error = y - P.Cm @ xhat  # can also use tilde vars (eq subtracts out)
        xhat_tilde = xhat - self.x_eq
        u_tilde = self.u_prev - self.u_eq
        xhat_dot = P.A @ xhat_tilde + P.B @ u_tilde + self.L @ y_error
        return xhat_dot

    def observer_rk4_step(self, y):
        k1 = self.observer_f(self.xhat_tilde, y)
        k2 = self.observer_f(self.xhat_tilde + P.ts / 2 * k1, y)
        k3 = self.observer_f(self.xhat_tilde + P.ts / 2 * k2, y)
        k4 = self.observer_f(self.xhat_tilde + P.ts * k3, y)
        xhat_tilde_dot = (k1 + 2 * k2 + 2 * k3 + k4) / 6
        self.xhat_tilde += xhat_tilde_dot * P.ts
        xhat = self.xhat_tilde + self.x_eq
        return xhat
