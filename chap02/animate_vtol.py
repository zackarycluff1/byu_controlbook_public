# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, F_vtol


# alias for parameters
P = F_vtol.params

# initialize signals for generating data
theta_gen = common.SignalGenerator(amplitude=np.pi, frequency=0.1)
phi_gen = common.SignalGenerator(amplitude=np.radians(45), frequency=0.1)
tau_gen = common.SignalGenerator(amplitude=5.0, frequency=0.5)

# initialize data storage
x0 = np.array([P.theta0, P.phi0, P.thetadot0, P.phidot0, P.hdot0, P.h0])
x_hist = [x0]
u_hist = []

# loop over time
time = np.arange(start=0, stop=20, step=P.ts, dtype=np.float64)
for t in time[1:]:
    # generate fake state and input data
    x = np.zeros_like(x0)
    x[0] = theta_gen.sin(t)
    x[1] = phi_gen.sin(t)
    # velocity info can be anything for animation
    x[2:] = [theta_gen.sawtooth(t), phi_gen.sawtooth(t), theta_gen.sawtooth(t), phi_gen.sawtooth(t)]

    # inputs can be anything for animation
    u = np.array([tau_gen.square(t), tau_gen.square(t)])

    # store data for visualization
    x_hist.append(x)
    u_hist.append(u)

# convert data to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

# visualize generated data
viz = F_vtol.Visualizer(time, x_hist, u_hist)
viz.animate()  # may need to play with arguments to achieve desired animation speed