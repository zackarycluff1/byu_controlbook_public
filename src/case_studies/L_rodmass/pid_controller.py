# 3rd-party
import numpy as np

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class RodMassControllerPID(ControllerBase):
    """
    PID controller for the rod-mass system.
    
    Can be used as PD controller (ki=0) or full PID controller.
    """

    def __init__(self, kp=0., kd=0., ki=0.):
        """
        Initialize PID controller with specified gains.
        
        Args:
            kp: Proportional gain
            kd: Derivative gain 
            ki: Integral gain
        """
        # self.kp = kp
        # self.ki = ki
        # self.kd = kd

        # Tuning Parameters`
        tr = 0.25
        zeta = 0.95
        self.ki = ki

        # system parameters
        b0 = 160
        a1, a0 = 16, 3.2
        # b0 = P.tf_num[-1]
        # a1, a0 = P.tf_den[-2:]

        # find gains
        wn = 0.5 * np.pi / (tr * np.sqrt(1 - zeta**2))
        alpha0 = wn**2
        alpha1 = 2 * zeta * wn
        self.kp = (alpha0 - a0) / b0
        self.kd = (alpha1 - a1) / b0
        print(f"{self.kp = :.3f}, {self.ki = :.3f}, {self.kd = :.3f}")

        # TODO: initialize necessary variables for your controller here ...
        # tr = 2        # Rise time (seconds) - how fast to reach target
        # zeta = 0.9    # Damping ratio (0.7 is good - no overshoot but fast)
        # wn = np.sqrt(3.2+160*kp)   # Natural frequency
        # alpha1 = 2 * zeta * wn
        # alpha0 = wn**2

        # b0 = 160
        # a1, a0 = 16, 3.2

        # Closed-loop: s^2 + (a1 + b0*kd)*s + (a0 + b0*kp) = s^2 + alpha1*s + alpha0
        # self.kp = (alpha0 - a0) / b0  # Proportional gain
        # self.kd = (alpha1 - a1) / b0  # Derivative gain
        # print(f"PD Gains: kp = {self.kp:.2f}, kd = {self.kd:.3f}")

        # dirty derivative variables
        self.sigma = 0.05
        self.beta = (2 * self.sigma - P.ts) / (2 * self.sigma + P.ts)
        self.thetadot_hat = P.thetadot0
        self.theta_prev = P.theta0

        # variables for integrator
        self.error_prev = 0.0
        self.error_integral = 0.0

        self.u_e = P.u_e


    # TODO: implement functions needed for your controller here ...
    def update_with_measurement(self, r, y):
        theta = y[0]
        theta_ref = r[0]

        # dirty derivative to estimate thetadot
        theta_diff = (theta - self.theta_prev) / P.ts
        self.thetadot_hat = self.beta * self.thetadot_hat + (1 - self.beta) * theta_diff
        self.theta_prev = theta

        # compute input from partially estimated state
        xhat = np.array([theta, self.thetadot_hat])

        # integrate error
        error = theta_ref - theta
        if (
            abs(self.thetadot_hat) < 0.08
        ):  # anti-windup: only integrate if thetadot is small
            self.error_integral += P.ts * (error + self.error_prev) / 2
        self.error_prev = error
        
        torque_tilde = (self.kp*error + self.ki*self.error_integral - self.kd*self.thetadot_hat)

        torque = torque_tilde + self.u_e

        u_unsat = np.array([torque])
        u = self.saturate(u_unsat, u_max=P.tau_max)

        return u, xhat

