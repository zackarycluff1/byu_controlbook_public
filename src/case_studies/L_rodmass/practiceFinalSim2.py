"""
Practice Final Exam - Part 2: Equilibrium Verification
Rod-Mass System

This script verifies that the equilibrium torque holds the system at rest.
Students implement dynamics in dynamics.py, then run this to verify.
"""
# 3rd-party
import numpy as np
import matplotlib.pyplot as plt

# local (controlbook)
from case_studies import common, L_rodmass



# Instantiate system and controller
system = L_rodmass.Dynamics(alpha=0.0)  # no parameter uncertainty
reference = common.SignalGenerator(amplitude=0.0)  # zero reference

# TODO: define the controller to use here (if you defined an equilibrium 
# controller, use it here to verify equilibrium): 

# controller = 

# Run simulation
time, x_hist, u_hist, r_hist, xhat_hist, *_ = common.run_simulation(
    system,
    [reference],
    controller,
    controller_input="measurement",
    t_final=5.0,
    dt=L_rodmass.params.ts
)

# Print verification results
# you can print values here that you need for the final exam. 

# Visualize results
viz = L_rodmass.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist)
viz.plot()
viz.animate()
