import numpy as np
 
# local (controlbook)
from case_studies import common, H_hummingbird
 
 
hummingbird = H_hummingbird.Dynamics()
controller = H_hummingbird.ControllerLonPD()
 
# square wave reference for theta: 30 deg amplitude, 0.1 Hz
# phi_ref is a flat zero signal just to occupy index 0 for correct plotting
phi_ref   = common.SignalGenerator(amplitude=0.0, frequency=0.1, y_offset=0.0)
theta_ref = common.SignalGenerator(amplitude=np.deg2rad(30), frequency=0.1, y_offset=0.0)
 
refs = [phi_ref, theta_ref]
 
time, x_hist, u_hist, r_hist, *_ = common.run_simulation(
    hummingbird,
    refs,
    controller,
    controller_input="state",
    t_final=50,
    dt=H_hummingbird.params.ts,
)
 
viz = H_hummingbird.Visualizer(time, x_hist, u_hist, r_hist)
viz.plot()
#viz.animate()