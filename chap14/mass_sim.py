# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, D_mass


mass = D_mass.Dynamics(alpha=0.2)
controller = D_mass.ControllerSSIO()
z_ref = common.SignalGenerator(amplitude=1, frequency=0.01)

z_noise = common.SignalGenerator(amplitude=0.001)

time, x_hist, u_hist, r_hist, xhat_hist, *_ = common.run_simulation(
    mass,
    [z_ref],
    controller,
    controller_input="measurement",
    input_disturbance=np.array([0.25]),
    output_noise=[z_noise],
    t_final=40,
    dt=D_mass.params.ts,
)

viz = D_mass.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist)
viz.plot()
# viz.animate()
