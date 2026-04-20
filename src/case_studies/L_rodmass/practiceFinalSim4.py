"""
Practice Final Exam - Part 4: State-Space Control with Observers
Rod-Mass System

This script demonstrates:
- Part 4.2: State-space control with integrator (observer-based)
- Part 4.5: Observer-based control with input disturbance
- Part 4.6: Observer estimation error analysis
- Part 4.7: LQR controller for faster response
"""
# 3rd-party
import numpy as np
import matplotlib.pyplot as plt

# local (controlbook)
from case_studies import common, L_rodmass


print("\n" + "="*60)
print("Part 4: State-Space Control with Observers")
print("="*60)

# Reference signal - 20 degree square wave at 0.1 Hz
reference = common.SignalGenerator(amplitude=20*np.pi/180, frequency=0.1)

#=========================================================================
# Part 4.2: State-Space Control with Integrator (Full State Feedback)
#=========================================================================
print("\n--- Part 4.2: State-Space Control (Full State) ---")

# TODO: Implement the state space integral controller
controller_ssi = L_rodmass.ControllerSSI()
# TODO: # Instantiate the system and state-space controller
system_ssi = L_rodmass.Dynamics(alpha=0.0)

# Run simulation (no input disturbance in this section)
time_ssi, x_ssi, u_ssi, r_ssi, xhat_ssi, d_ssi, dhat_ssi = common.run_simulation(
    system_ssi,
    [reference],
    controller_ssi,
    controller_input="state",
    t_final=20.0,
    dt=L_rodmass.params.ts
)


# Visualize (plots only - animation in final section)
viz_ssi = L_rodmass.Visualizer(time_ssi, x_ssi, u_ssi, r_ssi, xhat_ssi)
viz_ssi.plot()


#=========================================================================
# Parts 4.5 and 4.6: Observer-Based Control WITH Input Disturbance
#=========================================================================
print("\n--- Parts 4.5 and 4.6: Observer with Input Disturbance ---")

# TODO: Implement the state space integral controller with disturbance observer
controller_dist = L_rodmass.ControllerSSIDO()
# TODO: # Instantiate the system and controller with disturbance observer
system_dist = L_rodmass.Dynamics(alpha=0.1)
input_disturbance = common.SignalGenerator(amplitude=0.5*np.pi/180, frequency=0.1)

# Run simulation with disturbance
time_dist, x_dist, u_dist, r_dist, xhat_dist, d_dist, dhat_dist = common.run_simulation(
    system_dist,
    [reference],
    controller_dist,
    controller_input="measurement",
    input_disturbance=input_disturbance,
    t_final=20.0,
    dt=L_rodmass.params.ts
)


# Visualize with disturbance plots (plots only - animation in final section)
viz_dist = L_rodmass.Visualizer(time_dist, x_dist, u_dist, r_dist, xhat_dist, d_dist, dhat_dist)
viz_dist.plot()

#=========================================================================
# Part 4.7: LQR Controller for Faster Response
#=========================================================================
print("\n--- Part 4.7: LQR Controller ---")

# TODO: Implement an LQR controller with integral control, and a disturbance observer. 
controller_lqr = L_rodmass.lqr_controller()
# TODO: # Instantiate the system and lqr controller
system_lqr = L_rodmass.Dynamics(alpha=0.0)

# Run simulation with disturbance
time_lqr, x_lqr, u_lqr, r_lqr, xhat_lqr, d_lqr, dhat_lqr = common.run_simulation(
    system_lqr,
    [reference],
    controller_lqr,
    controller_input="measurement",
    input_disturbance=input_disturbance,
    t_final=20.0,
    dt=L_rodmass.params.ts
)


# Visualize LQR results
viz_lqr = L_rodmass.Visualizer(time_lqr, x_lqr, u_lqr, r_lqr, xhat_lqr, d_lqr, dhat_lqr)
viz_lqr.plot()
viz_lqr.animate()

print("\n" + "="*60)
print("State-Space Control Complete")
print("="*60)
