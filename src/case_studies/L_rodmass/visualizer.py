# 3rd-party
import numpy as np
from numpy.typing import NDArray

# local (controlbook)
from .animator import RodMassAnimator
from ..common import Visualizer


class RodMassVisualizer(Visualizer):
    """
    Visualizer for the rod-mass system.
    
    Creates static plots of states and inputs over time, and provides
    animation capability through the RodMassAnimator.
    
    States plotted:
        - θ (angle in degrees)
        - θ̇ (angular velocity in rad/s)
    
    Inputs plotted:
        - τ (torque in N⋅m)
    """

    def __init__(
        self,
        t_hist: NDArray[np.float64],
        x_hist: NDArray[np.float64],
        u_hist: NDArray[np.float64],
        r_hist: NDArray[np.float64] | None = None,
        xhat_hist: NDArray[np.float64] | None = None,
        d_hist: NDArray[np.float64] | None = None,
        dhat_hist: NDArray[np.float64] | None = None,
    ):
        """
        Initialize the visualizer with simulation history.
        
        Args:
            t_hist: Time history array
            x_hist: State history array (rows = time, cols = states)
            u_hist: Input history array
            r_hist: Reference history (optional)
            xhat_hist: State estimate history (optional, for observers)
            d_hist: Disturbance history (optional)
            dhat_hist: Disturbance estimate history (optional)
        """
        # Define labels for states and inputs (with units)
        # Note: Including "deg" in label triggers automatic rad→deg conversion
        x_labels = [
            r"$\theta$ (deg)",      # Angle (will be auto-converted from rad)
            r"$\dot{\theta}$ (rad/s)",  # Angular velocity
        ]
        u_labels = [r"$\tau$ (Nm)"]  # Torque
        
        # Call parent class constructor (handles all plotting logic)
        super().__init__(
            t_hist, x_hist, x_labels,
            u_hist, u_labels,
            r_hist, d_hist, xhat_hist, dhat_hist,
        )

    def get_system_animator(self, viz_object, x_hist, r_hist, blit=True):
        """
        Return the system-specific animator for animated visualization.
        
        Args:
            viz_object: Matplotlib axis object to draw on
            x_hist: State history to animate
            r_hist: Reference history (not used in rod-mass animation)
            blit: Use blitting for faster animation
            
        Returns:
            RodMassAnimator instance
        """
        from matplotlib.axes import Axes
        assert isinstance(viz_object, Axes)
        r_hist = None  # animation does not use reference
        ax = viz_object
        return RodMassAnimator(ax, x_hist, r_hist, blit)
