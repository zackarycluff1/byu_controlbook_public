"""
═══════════════════════════════════════════════════════════════════════════════
CONTROLLER TEMPLATE FOR STUDENTS
═══════════════════════════════════════════════════════════════════════════════

INSTRUCTIONS:
1. Copy this file to your case study folder (e.g., D_mass/pd_controller.py)
2. Rename class below to match your system (e.g., "MassControllerPD")
3. Fill in the TODOs below
4. Refer to A_arm/pd_controller.py for a working example

WHAT THIS FILE DOES:
This file implements the CONTROLLER - the math that decides what control
input to apply based on what you want (reference) vs. what you have (state or
measurement).

THREE IMPLEMENTATION TYPES BASED ON COURSE CHAPTER:
- Chapter 8:    update_with_state(r, x) using full state
- Chapters 9-10: update_with_measurement(r, y) using numerical derivative
- Chapters 12+:  update_with_measurement(r, y) using observer
═══════════════════════════════════════════════════════════════════════════════
"""

# 3rd-party or pip installed python libraries
import numpy as np

# local (controlbook) modules
from ..common import ControllerBase
from . import params as P


class AltitudeControllerPD(ControllerBase):
    """
    Example:
    PD (Proportional-Derivative) controller for Vtol.

    This controller computes: u = kp * error - kd * velocity
    where error = (desired position) - (actual position)
    """

    def __init__(self):
        """
        Initialize the controller and compute gains.

        The gains (kp and kd for a PD controller) determine how aggressively
        the controller responds.

        We compute gains in this init function (based on desired closed-loop
        poles or rise time).
        """
        # ═══════════════════════════════════════════════════════════════
        # TODO: Choose your design approach (pick ONE)
        # ═══════════════════════════════════════════════════════════════

        # APPROACH 1: Pole placement (Chapter 7)
        # Specify where you want the closed-loop poles
        # p1 = -3.0  # First pole (real part < 0 for stability!)
        # p2 = -4.0  # Second pole (real part < 0 for stability!)

        # # Compute desired characteristic equation: (s - p1)(s - p2) = s^2 + alpha1*s + alpha0
        # des_CE = np.poly([p1, p2])  # Get coefficients from desired poles
        # alpha1, alpha0 = des_CE[-2:]  # Pulling off coefficients of polynomial in "s"

        # APPROACH 2: Rise time + damping ratio (Chapter 8)
        # Easier to relate to physical performance!
        tr_h = 8.0        # Rise time (seconds) - how fast to reach target
        zeta = 0.707    # Damping ratio (0.7 is good - no overshoot but fast)
        wn_h = 2.2 / tr_h   # Natural frequency
        alpha1_h = 2 * zeta * wn_h
        alpha0_h = wn_h**2

        M = 10

        tr_th = 0.8
        wn_th = 2.2 / tr_th
        alpha1_th = 2 * zeta * wn_th
        alpha0_th = wn_th**2

        tr_z = M * tr_th
        wn_z = 2.2 / tr_z
        alpha1_z = 2 * zeta * wn_z
        alpha0_z = wn_z**2

        # ═══════════════════════════════════════════════════════════════
        # TODO: Get transfer function coefficients from params.py
        # ═══════════════════════════════════════════════════════════════
        # These come from your system's transfer function (Chapter 5):
        # G(s) = b0 / (s^2 + a1*s + a0)
        b0_h = P.tf_altitude_num[-1]  # Numerator coefficient
        a1_h, a0_h = P.tf_altitude_den[-2:]  # Denominator coefficients
        # Closed-loop: s^2 + (a1 + b0*kd)*s + (a0 + b0*kp) = s^2 + alpha1*s + alpha0
        self.kp_h = (alpha0_h - a0_h) / b0_h  # Proportional gain
        self.kd_h = (alpha1_h - a1_h) / b0_h  # Derivative gain
        print(f"Height PD Gains: kp = {self.kp_h:.2f}, kd = {self.kd_h:.3f}")
        # ═══════════════════════════════════════════════════════════════
        # Theta Kd and Kp


        b0_th = P.tf_theta_num[-1]
        a1_th, a0_th = P.tf_theta_den[-2:]

        # Closed-loop: s^2 + (a1 + b0*kd)*s + (a0 + b0*kp) = s^2 + alpha1*s + alpha0
        self.kp_th = (alpha0_th - a0_th) / b0_th  # Proportional gain
        self.kd_th = (alpha1_th - a1_th) / b0_th  # Derivative gain
        print(f"Theta PD Gains: kp = {self.kp_th:.2f}, kd = {self.kd_th:.3f}")
        # ═══════════════════════════════════════════════════════════════

        # ═══════════════════════════════════════════════════════════════
        # Z Kd and Kp

        b0_z = P.tf_z_num[-1]
        a1_z, a0_z = P.tf_z_den[-2:]

        # Closed-loop: s^2 + (a1 + b0*kd)*s + (a0 + b0*kp) = s^2 + alpha1*s + alpha0
        self.kp_z = (alpha0_z - a0_z) / b0_z  # Proportional gain
        self.kd_z = (alpha1_z - a1_z) / b0_z  # Derivative gain
        print(f"Z PD Gains: kp = {self.kp_z:.2f}, kd = {self.kd_z:.3f}")
        # ═══════════════════════════════════════════════════════════════

        # ═══════════════════════════════════════════════════════════════
        # Compute PD gains
        # ═══════════════════════════════════════════════════════════════
        # From control theory (see textbook derivation):

        # ═══════════════════════════════════════════════════════════════
        # TODO: Set equilibrium control input (for linearization)
        # ═══════════════════════════════════════════════════════════════
        # This is the control input needed to maintain equilibrium.
        # Often it's just 0, but for some systems (i.e. with gravity or
        # springs) the equilibrium input is nonzero.
        #
        # EXAMPLE (arm with gravity): self.u_eq = P.m * P.g * P.ell / 2

        self.u_eq = P.u_eq  # Get from params.py

    def update_with_state(self, r, x):
        """
        This function is defined in the base class ControllerBase, intially,
        but requires implementation here. We will compute the control input
        using full state feedback.

        This is called every time step during simulation.

        Args:
            r: Reference vector (variable that you want to track) - same size as output
            x: State vector - [position, velocity, ...]

        Returns:
            u: Control input vector (what to apply to the system) - [force, torque, ...]

        ### CONTROL LAW (PD):
            u = kp * error - kd * velocity + u_eq
        where:
            error = reference_position - actual_position
            u_eq = equilibrium control (accounts for gravity, etc.)
        """

        # ═══════════════════════════════════════════════════════════════
        # STEP 1: Unpack reference and state
        # ═══════════════════════════════════════════════════════════════
        # Reference: What you WANT the output to be
        position_ref = r[0]  # RENAME to match your system (z_ref, h_ref, theta_ref, etc.)
        altitude_ref = r[1]
        # State: What the system IS doing
        altitude = x[1]  # RENAME to match your system (z, theta, etc.)
        h_velocity = x[4]  # RENAME to match your system (zdot, thetadot, etc.)

        position = x[0]  # RENAME to match your system (z, theta, etc.)
        z_velocity = x[3]  # RENAME to match your system (zdot, thetadot, etc.)

        theta = x[2]  # RENAME to match your system (z, theta, etc.)
        theta_velocity = x[5]  # RENAME to match your system (zdot, thetadot, etc.)


        # ═══════════════════════════════════════════════════════════════
        # STEP 2: Compute error
        # ═══════════════════════════════════════════════════════════════
        z_error = position_ref - position
        h_error = altitude_ref - altitude

        # ═══════════════════════════════════════════════════════════════
        # STEP 3: Apply PD control law
        # ═══════════════════════════════════════════════════════════════
        # This is the "tilde" (deviation) control from equilibrium
        f_tilde = self.kp_h * h_error - self.kd_h * h_velocity

        theta_ref = self.kp_z * z_error - self.kd_z * z_velocity
        theta_error = theta_ref - theta
        tau_tilde = self.kp_th * theta_error - self.kd_th * theta_velocity 

        # Add equilibrium control (linearization point)
        u_total = f_tilde + self.u_eq[0]

        # ═══════════════════════════════════════════════════════════════
        # STEP 4: Return as array
        # ═══════════════════════════════════════════════════════════════
        u = np.array([u_total, tau_tilde])  # Must be array even if single value

        # OPTIONAL: Add saturation (Chapter 8)
        u = self.saturate(u, u_max=20)

        return u


# ═══════════════════════════════════════════════════════════════════════════════
# ADDITIONAL NOTES FOR STUDENTS
# ═══════════════════════════════════════════════════════════════════════════════
"""
WHY "MODIFIED" PD?
Regular PD:  u = kp*error + kd*error_dot
Modified PD: u = kp*error - kd*velocity

Modified PD is better because:
1. We can sometimes measure velocity directly (no need to differentiate error)
2. Avoids effect of zero in the transfer function numerator
3. More commonly used in practice

WHAT'S THE DIFFERENCE BETWEEN kp AND kd?
- kp (proportional gain): How hard to push based on HOW FAR you are from target
  - Too low: Slow response, never quite reaches target
  - Too high: Overshoots, oscillates
  
- kd (derivative gain): How hard to push based on HOW FAST you're moving
  - Too low: Lots of overshoot/oscillation
  - Too high: Sluggish response, sensitive to noise

TUNING TIPS:
1. Start with pole placement (p1=-2, p2=-3) or rise time (tr=2.0, zeta=0.707)
2. Simulate and look at the response
3. Too slow? Increase wn (decrease tr) or move poles further left
4. Overshooting? Increase zeta or move poles further from imaginary axis
5. Check that poles have negative real parts (stable!)

DEBUGGING:
- Controller not doing anything? Check that kp and kd are not zero or NaN
- System unstable? Check pole locations (should be left half-plane)
- Wrong direction? Check sign of error (should be ref - actual, not actual - ref)
- Not reaching target? Check u_eq
"""
