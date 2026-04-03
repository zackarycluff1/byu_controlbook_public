# 3rd party
import numpy as np
import control as cnt

from ..common.numeric_integration import rk4_step

class Observer():
    def __init__(self, params, des_obs_poles, x_hat0=None):
        # define system A, and B, and C
        self.A = params.A
        self.B = params.B
        self.Cm = params.Cm


        # desired poles
        # define a time step

        # check observability
            #get dimension of self.A to check obs
            # check rank of cnt.ctrl(self.A.T, self.Cm.T) using np.linalg

        # define L  
        # self.L = cnt.place(self.A.T, self.Cm.T, des_obs_poles).T
        
        # -  place poles

        # initialize self.x_hat
            # if x_hat0 = None      
                # need to define self.x_hat = 00000
            # else 
                # self.x_hat = x_hat_0

        # initialize self.u_prev .... needed in "observer_dynamics"
        


    def observer_dynamics(self, x_hat, y_m): # our normal dynamics have been f(x, u)

        # x_hat_dot = self.A @ x_hat + self.B @ self.u_prev + self.L @ (y_m - self.Cm @ x_hat)

    def update(self, y_m, u_prev):

        # self.u_prev = u_prev
        # self.x_hat = rk4_step(self.observer_dynamics, self.x_hat, y_m, self.params.ts)
        # return self.x_hat