import numpy as np

# local (controlbook)
from case_studies import common, H_hummingbird

hummingbird = H_hummingbird.Dynamics(alpha=0.2)
controller = H_hummingbird.ControllerFullPD()

# Disturbance: 0.1 N total force = 0.05 N on each motor
disturbance = np.array([0.05, 0.05])  # [u_left, u_right] in Newtons

# square wave reference for theta: 30 deg amplitude, 0.1 Hz
phi_ref = None
theta_ref = common.SignalGenerator(amplitude=np.radians(15), frequency=0.1, y_offset=0.0)
psi_ref = common.SignalGenerator(amplitude=np.radians(30), frequency=0.08, y_offset=0.0)

refs = [phi_ref, theta_ref, psi_ref]

time, x_hist, u_hist, r_hist, *_ = common.run_simulation(
    hummingbird,
    refs,
    controller,
    controller_input="measurement",
    t_final=50,
    dt=H_hummingbird.params.ts,
    input_disturbance=disturbance,
)

viz = H_hummingbird.Visualizer(time, x_hist, u_hist, r_hist)
viz.plot()