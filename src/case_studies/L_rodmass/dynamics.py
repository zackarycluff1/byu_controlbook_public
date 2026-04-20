# 3rd-party
import numpy as np

# local (controlbook)
from . import params as P
from ..common import DynamicsBase


class RodMassDynamics(DynamicsBase):
    """
    Rod-Mass system dynamics - point mass on massless rod with nonlinear spring.
    
    The system consists of a point mass m attached to a massless rod of length ell,
    connected to a wall with a nonlinear rotary spring and damper. 
    """

    def __init__(self, alpha=0.0):
        """
        Initialize the rod-mass system dynamics.
        
        Args:
            alpha: Parameter uncertainty scaling factor as a percentage. 
        """
        super().__init__(
            state0=np.array([P.theta0, P.thetadot0]),
            u_min=-P.tau_max,
            u_max=P.tau_max,
            dt=P.ts,
        )
        
        # Randomize physical parameters for uncertainty analysis
        self.m = self.randomize_parameter(P.m, alpha)
        self.ell = self.randomize_parameter(P.ell, alpha)
        self.k1 = self.randomize_parameter(P.k1, alpha)
        self.k2 = self.randomize_parameter(P.k2, alpha)
        self.b = self.randomize_parameter(P.b, alpha)
        self.g = P.g  # Gravity is well-known, don't randomize


    def f(self, x, u):
        """
        State evolution equation: xdot = f(x,u)
        
        Args:
            x: State vector 
            u: Input vector 
            
        Returns:
            xdot: State derivative 
        """

        # TODO: implement your dynamics here ... 
        theta, thetadot = x
        torque = u[0]

        friction = self.b * thetadot
        thetaddot = (torque - self.b*thetadot - self.m*self.g*self.ell*np.cos(theta) + self.k1*theta+self.k2*theta**3)/(self.m*self.ell**2)
        xdot = np.array([thetadot, thetaddot])
        return xdot

    def h(self):
        """
        Output equation: y = h(x)
        
        Returns:
            y: Output vector
        """
        # TODO: implement your output equation here ...
        theta= self.state[0]
        y = np.array([theta])
        return y
