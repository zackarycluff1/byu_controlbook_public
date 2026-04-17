# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, F_vtol


vtol = F_vtol.Dynamics()
controller = F_vtol.ControllerSSIDO()

z_ref = common.SignalGenerator(amplitude=4, frequency=0.05, y_offset=5)
h_ref = common.SignalGenerator(amplitude=3, frequency=0.03, y_offset=5)

refs = [z_ref, h_ref]

time, x_hist, u_hist, r_hist, xhat_hist, *_ = common.run_simulation(
    vtol,
    refs,
    controller,
    controller_input="measurement",
    input_disturbance=np.array([1.0, 0.1]),  # altitude and wind from F.14(a)
    output_noise=[
        common.SignalGenerator(amplitude=0.01),
        common.SignalGenerator(amplitude=0.01),
    ],
    t_final=50,
    dt=F_vtol.params.ts,
)

viz = F_vtol.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist)
viz.plot()
# viz.animate()
