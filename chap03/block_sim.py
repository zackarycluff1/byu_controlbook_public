# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, D_mass


# initialize system and input generator
mass = D_mass.Dynamics()
force_gen = common.SignalGenerator(amplitude=10.0, frequency=1)

# initialize data storage
x_hist = [mass.state]
u_hist = []

# loop over time
time = np.arange(start=0.0, stop=50.0, step=D_mass.params.ts, dtype=np.float64)
for t in time[1:]:
    # generate input signal
    u = np.array([force_gen.sin(t)])

    # simulate system response
    y = mass.update(u)

    # store data for visualization
    u_hist.append(u)
    x_hist.append(mass.state)

# convert data to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

# visualize
viz = D_mass.Visualizer(time, x_hist, u_hist)
viz.animate()  # could also just call viz.plot()
