"""
L_rodmass case study - Rod-Mass system with nonlinear spring
"""

# Core components (always available)
from .animator import RodMassAnimator as Animator
from .dynamics import RodMassDynamics as Dynamics
from .visualizer import RodMassVisualizer as Visualizer
from . import params

__all__ = ["Animator", "Dynamics", "Visualizer", "params"]

# Controllers (import with try/except for student version compatibility)
try:
    from .equi_controller import RodMassEquilibrium as ControllerEquilibrium
    __all__.append("ControllerEquilibrium")
except ImportError:
    pass
try:
    from .pid_controller import RodMassControllerPID as ControllerPID
    __all__.append("ControllerPID")
except ImportError:
    pass

try:
    from .ssi_controller import RodMassSSIController as ControllerSSI
    __all__.append("ControllerSSI")
except ImportError:
    pass

try:
    from .ssi_dist_obs_controller import RodMassSSIDOController as ControllerSSIDO
    __all__.append("ControllerSSIDO")
except ImportError:
    pass

try:
    from .lqr_controller import RodMassLQRController as ControllerLQR
    __all__.append("ControllerLQR")
except ImportError:
    pass
