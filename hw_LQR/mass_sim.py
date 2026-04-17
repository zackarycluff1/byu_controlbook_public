# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, D_mass


arm = D_mass.Dynamics(alpha=0.2)
controller = D_mass.ControllerLQRIDO(separate_integrator=False)
theta_ref = common.SignalGenerator(amplitude=np.radians(50), frequency=0.05)
d_force = np.array([0.5])
theta_noise = common.SignalGenerator(amplitude=0.001)

time, x_hist, u_hist, r_hist, xhat_hist, d_hist, dhat_hist = common.run_simulation(
    arm,
    [theta_ref],
    controller,
    controller_input="measurement",
    input_disturbance=d_force,
    output_noise=[theta_noise],
    t_final=20,
    dt=A_arm.params.ts,
)

viz = A_arm.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist, d_hist, dhat_hist)
viz.plot()
# viz.animate()
