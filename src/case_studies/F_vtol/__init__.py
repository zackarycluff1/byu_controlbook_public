"""
F_vtol case study - Vertical Takeoff and Landing aircraft

STUDENT VERSION - Files released incrementally throughout the semester.
Import errors are gracefully handled for not-yet-released components.
"""

# Core components (always available for animation/visualization)
from .animator import VTOLAnimator as Animator
from .visualizer import VTOLVisualizer as Visualizer
from . import params

__all__ = ["Animator", "Visualizer", "params"]

# Optional imports - automatically available as files are released
# Using try/except allows the package to work even when files are missing

try:
    from .dynamics import VtolDynamics as Dynamics

    __all__.append("Dynamics")
except ImportError:
    pass

try:
    from .altitude_pd_controller import AltitudeControllerPD

    __all__.append("AltitudeControllerPD")
except ImportError:
    pass

try:
    from .full_pd_controller import VTOLControllerPD as ControllerPD

    __all__.append("ControllerPD")
except ImportError:
    pass

try:
    from .pid_controller import VTOLControllerPID as ControllerPID

    __all__.append("ControllerPID")
except ImportError:
    pass

try:
    from .ss_controller import VTOLControllerSS as ControllerSS

    __all__.append("ControllerSS")
except ImportError:
    pass

try:
    from .ssi_controller import VTOLControllerSSI as ControllerSSI

    __all__.append("ControllerSSI")
except ImportError:
    pass

try:
    from .ssi_obs_controller import VTOLControllerSSIO as ControllerSSIO

    __all__.append("ControllerSSIO")
except ImportError:
    pass

try:
    from .ssi_dist_obs_controller import VTOLControllerSSIDO as ControllerSSIDO

    __all__.append("ControllerSSIDO")
except ImportError:
    pass

try:
    from .lqr_controller import VTOLControllerSSIDO as ControllerLQRIDO

    __all__.append("ControllerLQRIDO")
except ImportError:
    pass
