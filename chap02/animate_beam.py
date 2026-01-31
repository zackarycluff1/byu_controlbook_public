# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, E_blockbeam


# alias for parameters
P = E_blockbeam.params

# initialize signals for generating data
z_gen = common.SignalGenerator(amplitude=0.5, frequency=0.1)
theta_gen = common.SignalGenerator(amplitude=np.radians(45), frequency=0.5)
f_gen = common.SignalGenerator(amplitude=5.0, frequency=0.5)

# initialize data storage
x0 = np.array([P.z0, P.theta0, P.zdot0, P.thetadot0])
x_hist = [x0]
u_hist = []

# loop over time
time = np.arange(start=0, stop=20, step=P.ts, dtype=np.float64)
for t in time[1:]:
    # generate fake state and input data
    x = np.empty_like(x0)
    x[0] = z_gen.sin(t)
    x[1] = theta_gen.sin(t)
    x[2] = z_gen.sawtooth(t)  # velocity info can be anything for animation
    x[3] = theta_gen.sawtooth(t)  # velocity info can be anything for animation

    # inputs can be anything for animation
    u = np.array([f_gen.square(t)])

    # store data for visualization
    x_hist.append(x)
    u_hist.append(u)

# convert data to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

# visualize generated data
viz = E_blockbeam.Visualizer(time, x_hist, u_hist)
viz.animate()  # may need to play with arguments to achieve desired animation speed
