# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class RodMassSSIController(ControllerBase):
    """
    State-space integral control for rod-mass system. 
    
    Uses:
    - Full state feedback with integral action
    - No observer (this is for Part 4.1-4.2 of practice final)
    """

    def __init__(self, separate_integrator=True):
        """
        Initialize controller with pole placement design.
        
        """
        tr = .25
        zeta = 0.95
        integrator_pole = [-10.0]

        A = np.array([[0, 1], [-P.k1/(P.m*P.ell**2), -0.1/(P.m*P.ell**2)]])
        B = np.array([[0], [1/(P.m*P.ell**2)]])
        

        self.C_r = np.array([[1, 0]])

        # augmented system
        A1 = np.block([[A, np.zeros((2, 1))], [-self.C_r, np.zeros(1)]])
        B1 = np.vstack((B, 0))

        # check controllability
        if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 3:
            raise ValueError("System not controllable")

        # compute gains
        wn = 0.5 * np.pi / (tr * np.sqrt(1 - zeta**2))
        des_char_poly = [1, 2 * zeta * wn, wn**2]
        des_sys_poles = np.roots(des_char_poly)
        des_poles = np.hstack((des_sys_poles, integrator_pole))
        self.K1 = cnt.place(A1, B1, des_poles)
        self.K = self.K1[:, :2]
        self.ki = self.K1[:, 2:]
        print("des_poles:", des_poles)
        print("K1:", self.K1)
        print("K:", self.K)
        print("ki:", self.ki)

        # TODO: initialize necessary variables for your controller here ...
        
        # linearization point
        self.x_eq = np.zeros(2)
        self.r_eq = self.C_r @ self.x_eq
        self.x1_eq = np.hstack((self.x_eq, 0))

        # dirty derivative variables
        sigma = 0.05  # cutoff freq for dirty derivative
        self.beta = (2 * sigma - P.ts) / (2 * sigma + P.ts)
        self.thetadot_hat = P.thetadot0
        self.theta_prev = P.theta0

        # integrator variables
        self.error_prev = 0.0
        self.error_integral = 0.0
        self.separate_integrator = separate_integrator

        # # compute gains
        # wn = 0.5 * np.pi / (tr * np.sqrt(1 - zeta**2))
        # des_char_poly = [1, 2 * zeta * wn, wn**2]
        # des_poles = np.roots(des_char_poly)
        # self.K = cnt.place(P.A, P.B, des_poles)
        # self.kr = -1.0 / (P.Cr @ np.linalg.inv(P.A - P.B @ self.K) @ P.B)
        # print("des_poles:", des_poles)
        # print("K:", self.K)
        # print("kr:", self.kr)
    # TODO: implement functions needed for your controller here ...
    def update_with_state(self, r, x):
        # calculate x _hat using our observer
        #replace every instance of x with x_hat
        # convert to linearization (tilde) variables

        x_tilde = x - self.x_eq

        # integrate error
        error = r - self.C_r @ x  # can also use tilde vars (eq subtracts out)
        self.error_integral += P.ts * (error + self.error_prev) / 2
        self.error_prev = error

        # compute feedback control
        if self.separate_integrator:
            u_tilde = -self.K @ x_tilde - self.ki @ self.error_integral
        else:
            x1_tilde = np.hstack((x_tilde, self.error_integral))
            u_tilde = -self.K1 @ x1_tilde

        # convert back to original variables (feedback linearization)
        theta = x[0]
        u_fl = 0.245
        u_unsat = u_tilde + u_fl
        u = self.saturate(u_unsat, u_max=P.tau_max)
        return u