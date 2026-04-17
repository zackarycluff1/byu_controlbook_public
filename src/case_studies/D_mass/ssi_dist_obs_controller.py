# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase

#include this line so you don't have to 
from ..common.numeric_integration import rk4_step


class MassSSIDOController(ControllerBase):
    def __init__(self, separate_integrator=True):
        # control tuning parameters
        tr = 2
        zeta = 0.707
        integrator_pole = [-5.0]

        # augmented system
        A1 = np.block([[P.A, np.zeros((2, 1))], [-P.Cr, np.zeros(1)]])
        B1 = np.vstack((P.B, 0))

        # check controllability
        if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 3:
            raise ValueError("System not controllable")

        # compute control gains
        wn = 2.2 / tr  # assumes zeta = 0.707
        des_char_poly = [1, 2 * zeta * wn, wn**2]
        des_sys_poles = np.roots(des_char_poly)
        des_poles = np.hstack((des_sys_poles, integrator_pole))
        self.K1 = cnt.place(A1, B1, des_poles)
        self.K = self.K1[:, :2]
        self.ki = self.K1[:, 2:]

        # linearization point
        self.x_eq = P.x_eq
        self.r_eq = P.Cr @ self.x_eq
        self.x1_eq = np.hstack((self.x_eq, 0))

        # integrator variables
        self.error_prev = 0.0
        self.error_integral = 0.0
        self.separate_integrator = separate_integrator

        ####### OBSERVER DESIGN ########
        # observer tuning parameters
        M_obs = 10.0  # time scale separation between controller and observer
        tr_obs = tr / M_obs
        zeta_obs = 0.707
        disturbance_pole = [-5.0]

        # augmented system
        self.A2 = np.block([[P.A, P.B], [np.zeros((1, 3))]])
        self.B2 = np.block([[P.B], [np.zeros(1)]])
        self.C2 = np.block([P.Cm, np.zeros(1)])

        # check observability
        if np.linalg.matrix_rank(cnt.ctrb(P.A.T, P.Cm.T)) != 2:
            raise ValueError("System not observable")

        # compute observer gain matrix
        wn_obs = 2.2 / tr_obs
        obs_char_poly = [1, 2 * zeta_obs * wn_obs, wn_obs**2]
        obs_sys_poles = np.roots(obs_char_poly)
        obs_poles = np.hstack((obs_sys_poles, disturbance_pole))
        self.L2 = cnt.place(self.A2.T, self.C2.T, obs_poles).T
        print("L2^T:", self.L2.T)

        # observer variables
        self.x2hat_tilde = np.zeros(3)
        self.u_prev = np.zeros(1)
        self.x2_eq = np.hstack((self.x_eq, [0]))

    def update_with_measurement(self, r, y):
        # update the observer with the measurement
        #use the equation we imported above here
        xhat, dhat = self.observer_rk4_step(y)

        # convert to linearization (tilde) variables
        x_tilde = xhat - self.x_eq

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

        # convert back to original variables (for feedback linearization)
        x = xhat[0]
        u_fl = P.u_eq
        u_unsat = u_tilde + u_fl
        u = self.saturate(u_unsat, u_max=P.force_max)

        # save the previous control input for the observer
        self.u_prev = u

        return u, xhat, dhat

    def observer_f(self, x2hat_tilde, y):
        x2hat = x2hat_tilde + self.x2_eq
        y_error = y - self.C2 @ x2hat  # can also use tilde vars (eq subtracts out)
        # xhat_tilde = xhat - self.x_eq
        u_fl = P.u_eq
        u_tilde = self.u_prev - u_fl
        x2hat_dot = self.A2 @ x2hat_tilde + self.B2 @ u_tilde + self.L2 @ y_error
        return x2hat_dot

    def observer_rk4_step(self, y):
        k1 = self.observer_f(self.x2hat_tilde, y)
        k2 = self.observer_f(self.x2hat_tilde + P.ts / 2 * k1, y)
        k3 = self.observer_f(self.x2hat_tilde + P.ts / 2 * k2, y)
        k4 = self.observer_f(self.x2hat_tilde + P.ts * k3, y)
        x2hat_tilde_dot = (k1 + 2 * k2 + 2 * k3 + k4) / 6
        self.x2hat_tilde += x2hat_tilde_dot * P.ts
        # xhat = self.xhat_tilde + self.x_eq
        # unpack estimated state and disturbance
        xhat_tilde = self.x2hat_tilde[:-1]
        xhat = xhat_tilde + self.x_eq
        dhat = self.x2hat_tilde[-1:]
        return xhat, dhat.copy()
