# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, D_mass


mass = D_mass.Dynamics()
controller = D_mass.ControllerPD()
z_ref = common.SignalGenerator(amplitude=np.radians(30), frequency=0.01)

time, x_hist, u_hist, r_hist, *_ = common.run_simulation(
    mass,
    [z_ref],
    controller,
    controller_input="state",
    t_final=5,
    dt=D_mass.params.ts,
)

viz = D_mass.Visualizer(time, x_hist, u_hist, r_hist)
viz.animate()
