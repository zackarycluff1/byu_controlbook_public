"""
Practice Final Exam - Part 3: PID Control
Rod-Mass System

This script demonstrates PID control with and without parameter uncertainty.
- First run: PD control (no integrator needed) with nominal parameters
- Second run: PID control with 10% parameter uncertainty
"""

# 3rd-party
import numpy as np
import matplotlib.pyplot as plt

# local (controlbook)
from case_studies import common, L_rodmass


print("\n" + "="*60)
print("Part 3: PID Control")
print("="*60)

# Reference signal - 20 degree square wave at 0.1 Hz
reference = common.SignalGenerator(amplitude=20*np.pi/180, frequency=0.1)

# TODO: Implement the PID controller (or PD and then PID controller)
# TODO: Instantiate system with nominal parameters and define the controller to use here (PD control for nominal case):


# Run simulation
time_pd, x_pd, u_pd, r_pd, xhat_pd, *_ = common.run_simulation(
    system_pd,
    [reference],
    controller_pd,
    controller_input="measurement",
    t_final=20.0,
    dt=L_rodmass.params.ts
)


# Visualize (plots only - animation in final section)
viz_pd = L_rodmass.Visualizer(time_pd, x_pd, u_pd, r_pd, xhat_pd)
viz_pd.plot()


#=========================================================================
# Part 3.6: PID Control with 10% Parameter Uncertainty
#=========================================================================
print("\n--- Part 3.6: PID Control with 10% Uncertainty ---")

# TODO: Instantiate system with 10% parameter uncertainty and define the controller to use here (PID control for uncertain case):

# Visualize
viz_pid = L_rodmass.Visualizer(time_pid, x_pid, u_pid, r_pid, xhat_pid)
viz_pid.plot()
viz_pid.animate()

print("\n" + "="*60)
print("PID Control Complete")
print("="*60)
