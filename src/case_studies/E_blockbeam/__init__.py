"""
E_blockbeam case study - Ball and beam system

STUDENT VERSION - Files released incrementally.
Import errors are silently ignored for not-yet-released files.
"""

# Core components (always available)
from .animator import BlockbeamAnimator as Animator
from .visualizer import BlockbeamVisualizer as Visualizer
from . import params

__all__ = ["Animator", "Visualizer", "params"]

# Optional imports using try/except
# These will be available as the course progresses

try:
    from .dynamics import BlockbeamDynamics as Dynamics

    __all__.append("Dynamics")
except ImportError:
    pass

try:
    from .pd_controller import BlockbeamControllerPD as ControllerPD

    __all__.append("ControllerPD")
except ImportError:
    pass

try:
    from .pid_controller import BlockbeamControllerPID as ControllerPID

    __all__.append("ControllerPID")
except ImportError:
    pass

try:
    from .ss_controller import BlockbeamSSController as ControllerSS

    __all__.append("ControllerSS")
except ImportError:
    pass

try:
    from .ssi_controller import BlockbeamSSIController as ControllerSSI

    __all__.append("ControllerSSI")
except ImportError:
    pass

try:
    from .ssi_obs_controller import BlockbeamSSIOController as ControllerSSIO

    __all__.append("ControllerSSIO")
except ImportError:
    pass

try:
    from .ssi_dist_obs_controller import BlockbeamSSIDOController as ControllerSSIDO

    __all__.append("ControllerSSIDO")
except ImportError:
    pass

try:
    from .lqr_controller import BlockbeamSSIDOController as ControllerLQRIDO

    __all__.append("ControllerLQRIDO")
except ImportError:
    pass

try:
    from .loopshaped_controller import (
        BlockbeamControllerLoopshaped as ControllerLoopshaped,
    )

    __all__.append("ControllerLoopshaped")
except ImportError:
    pass

try:
    from .loopshaping import design_loopshaped_controller

    __all__.append("design_loopshaped_controller")
except ImportError:
    pass
