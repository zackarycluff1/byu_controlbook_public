# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class RodMassSSIDOController(ControllerBase):
    """
    State-space integral control with disturbance observer for rod-mass system.
    """

    def __init__(self):
        """
        Initialize controller and observer with pole placement design.
        """

        # TODO: initialize necessary variables for your controller here ...

    # TODO: implement functions needed for your controller here ...
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