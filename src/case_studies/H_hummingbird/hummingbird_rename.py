# 3rd-party
import numpy as np
import case_studies.H_hummingbird.params as P
# local (controlbook)
from case_studies import common, H_hummingbird



phi_gen = common.SignalGenerator(amplitude=0.1, frequency=1.0, y_offset=0.01)
theta_gen = common.SignalGenerator(amplitude=0.1, frequency=1.0, y_offset=0.01)
psi_gen = common.SignalGenerator(amplitude=0.1, frequency=1.0, y_offset=0.01)
# initialize system and input generator
# vtol1 = F_vtol.Dynamics()
rotor_r_gen = common.SignalGenerator(amplitude=1.0, frequency=1.0, y_offset=0.0)
rotor_l_gen = common.SignalGenerator(amplitude=1.0, frequency=1.0, y_offset=0.0)

# initialize data storage
x = np.zeros(6)
x_hist = [x]
u_hist = []

# loop over time
time = np.arange(start=0.0, stop=20.0, step=H_hummingbird.params.ts, dtype=np.float64)
for t in time[1:]:
    # generate input signal
    # u = np.array([force_gen.sin(t), tau_gen.sin(t)])

    x0 = np.empty_like(x)
    x0[0] = phi_gen.sin(t)
    x0[1] = theta_gen.sin(t)
    x0[2] = psi_gen.sin(t)

    u = np.array([rotor_r_gen.sin(t),
                rotor_l_gen.sin(t)])          # (2,1)

    # u = (P.mixing @ U).squeeze()        # (2,)


    # simulate system
    # y = x0.update(u)

    # store data for visualization
    u_hist.append(u)
    x_hist.append(x0)

# convert data to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

# visualize
viz = H_hummingbird.Visualizer(time, x_hist, u_hist)
viz.animate()  # could also just call viz.plot()
