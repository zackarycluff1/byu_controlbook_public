# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, E_blockbeam


blockbeam = E_blockbeam.Dynamics(alpha=0.2)
controller = E_blockbeam.ControllerLQRIDO(separate_integrator=True)

z_ref = common.SignalGenerator(amplitude=0.5, frequency=0.04)
refs = [z_ref]

d_force = np.array([0.5])

z_noise = common.SignalGenerator(amplitude=0.001)
theta_noise = common.SignalGenerator(amplitude=0.001)
y_noise = [z_noise, theta_noise]

time, x_hist, u_hist, r_hist, xhat_hist, d_hist, dhat_hist = common.run_simulation(
    blockbeam,
    refs,
    controller,
    controller_input="measurement",
    input_disturbance=d_force,
    output_noise=y_noise,
    t_final=60,
    dt=B_pendulum.params.ts,
)

viz = B_pendulum.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist, d_hist, dhat_hist)
viz.plot()
# viz.animate()
