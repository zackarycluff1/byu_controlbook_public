# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class BlockbeamSSIDOController(ControllerBase):
    def __init__(self, separate_integrator=True):
        # tuning parameters
        # tr_theta = 0.25  # tuned for slower, more stable response.
        # # May be necessary due to initial value for theta (10 degrees).
        # zeta_theta = 0.707
        # M = 3
        # tr_z = tr_theta * M
        # zeta_z = 0.707
        # integrator_pole = [-5.0]
        Q = np.diag([1.0, 1.0, 1.0, 1.0, 5.0])
        R = np.array([[0.1]])

        # augmented system
        A1 = np.block([[P.A, np.zeros((4, 1))], [-P.Cr, np.zeros(1)]])
        B1 = np.vstack((P.B, 0))

        # check controllability
        if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 5:
            raise ValueError("System not controllable")

        # compute gains
        # wn_theta = 0.5 * np.pi / (tr_theta * np.sqrt(1 - zeta_theta**2))
        # theta_char_poly = [1, 2 * zeta_theta * wn_theta, wn_theta**2]
        # theta_poles = np.roots(theta_char_poly)

        # wn_z = 0.5 * np.pi / (tr_z * np.sqrt(1 - zeta_z**2))
        # z_char_poly = [1, 2 * zeta_z * wn_z, wn_z**2]
        # z_poles = np.roots(z_char_poly)

        # des_poles = np.hstack([theta_poles, z_poles, integrator_pole])

        # self.K1 = cnt.place(A1, B1, des_poles)
        # self.K = self.K1[:, :4]
        # self.ki = self.K1[:, 4:]
        self.K1, *_ = cnt.lqr(A1, B1, Q, R)
        self.K = self.K1[:, :4]
        self.ki = self.K1[:, 4:]
        if separate_integrator:
            print("K:", self.K)
            print("ki:", self.ki)
        else:
            print("K1:", self.K1)

        # linearization point
        self.x_eq = P.x_eq
        self.r_eq = P.Cr @ self.x_eq
        self.u_eq = P.u_eq

        # integrator variables
        self.error_prev = 0.0
        self.error_integral = 0.0
        self.separate_integrator = separate_integrator

        ####### OBSERVER DESIGN ########
        # observer tuning parameters
        M_obs = 10.0  # time scale separation between controller and observer
        tr_theta_obs = 0.05
        zeta_theta_obs = 0.9
        tr_z_obs = 0.15
        zeta_z_obs = 0.9
        disturbance_pole = [-1.0]

        # augmented system
        self.A2 = np.block([[P.A, P.B], [np.zeros((1, 5))]])
        self.B2 = np.block([[P.B], [np.zeros(1)]])
        self.C2 = np.block([P.Cm, np.zeros((2, 1))])

        # check observability
        if np.linalg.matrix_rank(cnt.ctrb(self.A2.T, self.C2.T)) != 5:
            raise ValueError("System not observable")

        # compute observer gain matrix
        wn_theta_obs = 2.2 / tr_theta_obs
        theta_obs_char_poly = [1, 2 * zeta_theta_obs * wn_theta_obs, wn_theta_obs**2]
        theta_obs_poles = np.roots(theta_obs_char_poly)

        wn_z_obs = 2.2 / tr_z_obs
        z_obs_char_poly = [1, 2 * zeta_z_obs * wn_z_obs, wn_z_obs**2]
        z_obs_poles = np.roots(z_obs_char_poly)

        obs_poles = np.hstack((theta_obs_poles, z_obs_poles))
        self.L = cnt.place(P.A.T, P.Cm.T, obs_poles).T
        print("L^T:", self.L.T)

        # observer variables
        self.xhat_tilde = np.zeros(4)
        self.u_prev = np.zeros(1)


    def update_with_measurement(self, r, y):
        # update the observer with the measurement
        xhat = self.observer_rk4_step(y)
        # Compute state-dependent u_eq using observer estimate

        # convert to linearization (tilde) variables
        x_tilde = xhat - self.x_eq
        r_tilde = r - self.r_eq

        # integrate error
        error = r - P.Cr @ x_tilde  # can also use tilde vars (eq subtracts out)
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
        u = self.saturate(u_unsat, u_max=P.force_max)

        # save the previous control input for the observer
        self.u_prev = u

        return u, xhat

    def observer_f(self, xhat, y):
        y_error = y - P.Cm @ xhat
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
