# 3rd-party pip installed packages
import numpy as np

# local (controlbook)
from case_studies import common, E_blockbeam


block = E_blockbeam.Dynamics()
controller = E_blockbeam.ControllerPD()

# this z_ref amplitude could be non-zero, but book asks for stabilization only
z_ref = common.SignalGenerator(amplitude=0.15, frequency=0.05, y_offset=0.25)
theta_ref = None
refs = [z_ref, theta_ref]

time, x_hist, u_hist, r_hist, *_ = common.run_simulation(
    block,
    refs,
    controller,
    controller_input="state",
    t_final=20,
    dt=E_blockbeam.params.ts,
)

viz = E_blockbeam.Visualizer(time, x_hist, u_hist, r_hist)
# viz.plot()
viz.animate()
