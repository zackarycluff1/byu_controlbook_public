# 3rd-party
import numpy as np
import case_studies.F_vtol.params as P
# local (controlbook)
from case_studies import common, F_vtol


# initialize system and input generator
vtol1 = F_vtol.Dynamics()
force_gen = common.SignalGenerator(amplitude=0.5, frequency=1.0, y_offset=14.715)
tau_gen = common.SignalGenerator(amplitude=0.001, frequency=1.0, y_offset=-0.01)

# initialize data storage
x_hist = [vtol1.state]
u_hist = []

# loop over time
time = np.arange(start=0.0, stop=25.0, step=F_vtol.params.ts, dtype=np.float64)
for t in time[1:]:
    # generate input signal
    u = np.array([force_gen.sin(t), tau_gen.sin(t)])

    # simulate system
    y = vtol1.update(u)

    # store data for visualization
    u_hist.append(u)
    x_hist.append(vtol1.state)

# convert data to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)
print("Number of dimensions: %d" % u_hist.ndim)

# visualize
viz = F_vtol.Visualizer(time, x_hist, u_hist)
viz.animate()  # could also just call viz.plot()
