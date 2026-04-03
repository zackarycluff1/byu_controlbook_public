import numpy as np

##### Chapter 2
# Physical parameters
m = 5  # mass of the arm (kg)
g = 9.8  # gravity (m/s**2)
b = 0.5  # damping coefficient (Nms)
k = 3

##### Chapter 3
# Initial Conditions
z0 = 0.0  # (rad)
zdot0 = 0.0  # (rad/s)

# Simulation parameters
t0 = 0.0  # start time
tf = 50.0  # end time
ts = 0.01  # integration time step

tf_num = [1/m]
tf_den = [1, b/m, k/m]

force_max = 6.0

u_eq = 0
x_eq = np.zeros(2)

A = np.array([[0, 1], [-k/m, -b/m]])
B = np.array([[0], [1/m]])
Cr = np.array([1, 0])
Cm = Cr