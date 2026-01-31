# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, D_mass


# initialize signals for generating data
theta_gen = common.SignalGenerator(amplitude=np.pi, frequency=0.1)
tau_gen = common.SignalGenerator(amplitude=5, frequency=0.5)

# initialize data storage
x0 = np.zeros(2)
x_hist = [x0]
u_hist = []

# loop over time
time = np.arange(start=0, stop=20, step=D_mass.params.ts, dtype=np.float64)
for t in time[1:]:
    # generate fake state and input data
    x = np.empty(2)
    x[0] = theta_gen.sin(t)
    x[1] = theta_gen.random(t)  # velocity info can be anything for animation

    # inputs can be anything for animation
    u = np.array([tau_gen.sawtooth(t)])

    # store data for visualization
    x_hist.append(x)
    u_hist.append(u)

# convert data to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

# visualize generated data
viz = D_mass.Visualizer(time, x_hist, u_hist)
viz.animate()  # may need to play with arguments to achieve desired animation speed
