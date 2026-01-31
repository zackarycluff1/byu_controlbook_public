# 3rd-party
import numpy as np
from numpy.typing import NDArray
from matplotlib.axes import Axes

# local (controlbook)
from ..common import Visualizer
from .animator import VTOLAnimator
from . import params as P


class VTOLVisualizer(Visualizer):
    def __init__(
        self,
        # TODO: decide whether to type annotate here
        t_hist: NDArray[np.float64],
        x_hist: NDArray[np.float64],
        u_hist: NDArray[np.float64],
        r_hist: NDArray[np.float64] | None = None,
        xhat_hist: NDArray[np.float64] | None = None,
        d_hist: NDArray[np.float64] | None = None,
        dhat_hist: NDArray[np.float64] | None = None,
        convert_to_F_tau: bool = False, # Make this false
    ):
        # NOTE: base Visualizer class will convert data to degrees for plotting
        # if "deg" is in the label
        x_labels = [
            "z (m)",
            "h (m)",
            r"$\theta$ (deg/s)",
            r"$\dot{z}$ (m/s)",
            r"$\dot{h}$ (m/s)",
            r"$\dot{\theta}$ (rad/s)",
        ]
        if convert_to_F_tau:
            u_hist = P.unmixer @ u_hist.T
            if d_hist is not None:
                d_hist = P.unmixer @ d_hist.T
            if dhat_hist is not None:
                dhat_hist = P.unmixer @ dhat_hist.T

            u_labels = ["F (N)", r"$\tau$ (Nm)"]
        else:
            u_labels = [r"$f_r$ (N)", r"$f_l$ (N)"]

        super().__init__(
            t_hist,
            x_hist,
            x_labels,
            u_hist,
            u_labels,
            r_hist,
            d_hist,
            xhat_hist,
            dhat_hist,
        )

    def get_system_animator(
        self,
        viz_object: Axes,
        x_hist: NDArray[np.float64],
        r_hist: NDArray[np.float64] | None,
        blit=True,
    ):
        assert isinstance(viz_object, Axes)
        ax = viz_object
        return VTOLAnimator(ax, x_hist, r_hist, blit)
