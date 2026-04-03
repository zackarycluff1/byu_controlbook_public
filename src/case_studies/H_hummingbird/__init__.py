"""
H_hummingbird case study - Hummingbird quadrotor

STUDENT VERSION - Files released incrementally throughout the semester.
Import errors are gracefully handled for not-yet-released components.
"""

# Core components (always available for animation/visualization)
from .animator import HummingbirdAnimator as Animator
from .visualizer import HummingbirdVisualizer as Visualizer
from . import params

__all__ = ["Animator", "Visualizer", "params"]

# Optional imports - automatically available as files are released
# Using try/except allows the package to work even when files are missing

try:
    from .dynamics_h3 import HummingbirdDynamics_h3 as Dynamics_h3
    __all__.append("Dynamics_h3")
except ImportError:
    pass

try:
    from .dynamics import HummingbirdDynamics as Dynamics
    __all__.append("Dynamics")
except ImportError:
    pass

try:
    from .longitudinal_pd_controller import HummingbirdControllerLonPD as ControllerLonPD
    __all__.append("ControllerLonPD")
except ImportError:
    pass

try:
    from .full_pd_controller_2 import HummingbirdControllerFullPD as ControllerFullPD
    __all__.append("ControllerFullPD")
except ImportError:
    pass

try:
    from .pid_controller import HummingbirdControllerPID as ControllerPID
    __all__.append("ControllerPID")
except ImportError:
    pass

try:
    from .ss_controller import HummingbirdControllerSS as ControllerSS
    __all__.append("ControllerSS")
except ImportError:
    pass

try:
    from .ssi_controller import HummingbirdControllerSSI as ControllerSSI
    __all__.append("ControllerSSI")
except ImportError:
    pass

try:
    from .ssi_obs_controller import HummingbirdControllerSSIO as ControllerSSIO
    __all__.append("ControllerSSIO")
except ImportError:
    pass

try:
    from .ssi_dist_obs_controller import HummingbirdControllerSSIDO as ControllerSSIDO
    __all__.append("ControllerSSIDO")
except ImportError:
    pass

try:
    from .lqr_controller import HummingbirdControllerSSIDO as ControllerLQRIDO
    __all__.append("ControllerLQRIDO")
except ImportError:
    pass
