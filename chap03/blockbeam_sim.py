# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, E_blockbeam


# initialize system and input generator
blockbeam = E_blockbeam.Dynamics()
force_gen = common.SignalGenerator(amplitude=0.5, frequency=1.0, y_offset=11.5)

# initialize data storage
x_hist = [blockbeam.state]
u_hist = []

# loop over time
time = np.arange(start=0.0, stop=15.0, step=E_blockbeam.params.ts, dtype=np.float64)
for t in time[1:]:
    # generate input signal
    u = np.array([force_gen.sin(t)])

    # simulate system response
    y = blockbeam.update(u)

    # store data for visualization
    u_hist.append(u)
    x_hist.append(blockbeam.state)

# convert data to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

# visualize
viz = E_blockbeam.Visualizer(time, x_hist, u_hist)
viz.animate()  # could also just call viz.plot()
