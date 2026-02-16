# 3rd-party
import numpy as np
import case_studies.H_hummingbird.params as P
# local (controlbook)
from case_studies import common, H_hummingbird


# initialize system and input generator
hum1 = H_hummingbird.Dynamics()
force_left_gen = common.SignalGenerator(amplitude=0.0, frequency=1.0, y_offset=0.0)
force_right_gen = common.SignalGenerator(amplitude=0.0, frequency=1.0, y_offset=0.50)

# initialize data storage
x_hist = [hum1.state]
u_hist = []

# loop over time
time = np.arange(start=0.0, stop=15.0, step=H_hummingbird.params.ts, dtype=np.float64)
for t in time[1:]:
    # generate input signal
    # u = np.array([force_gen.sin(t), tau_gen.sin(t)])

    fl_cmd = force_left_gen.sin(t)
    fr_cmd = force_right_gen.sin(t)
    #fl_cmd = P.m3 * P.g / 2
    #fr_cmd = P.m3 * P.g / 2


    U = np.array([[fl_cmd],
                [fr_cmd]])          # (2,1)

    u = U.squeeze()        # (2,)


    # simulate system
    y = hum1.update(u)

    # store data for visualization
    u_hist.append(u)
    x_hist.append(hum1.state)

# convert data to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

# visualize
viz = H_hummingbird.Visualizer(time, x_hist, u_hist)
viz.animate()  # could also just call viz.plot()
