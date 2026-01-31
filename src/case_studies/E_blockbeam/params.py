import numpy as np

##### Chapter 2
# Physical parameters
m1 = 0.35  # mass of the pendulum (kg)
m2 = 2.0  # mass of the cart (kg)
ell = 0.5  # length of the rod (m)
g = 9.81  # gravity (m/s**2)
b = 0.00  # damping coefficient (Ns)

##### Chapter 3
# Initial Conditions
z0 = 0.0  #  (m)
theta0 = np.radians(0.0)  # (rad)
zdot0 = 0.0  # (m/s)
thetadot0 = 0.0  # (rad/s)
h0 = 0.0
hdot0 = 0.0

# Simulation parameters
t0 = 0.0  # start time
tf = 50.0  # end time
ts = 0.01  # integration time step

##### Chapter 4
# Linearization/equilibrium point
x_eq = np.zeros(4)
u_eq = np.array([0.0])

##### Chapter 5
# Transfer function numerator and denominator
mass = m1 + m2
temp = m1 * ell / 6.0 + m2 * 2 * ell / 3.0
tf_inner_num = [-1 / temp]
tf_inner_den = [1, 0, -mass * g / temp]
tf_outer_num = [-2 * ell / 3, 0, g]
tf_outer_den = [1, 0, 0]

##### Chapter 6 / 11-14
# State space
temp = m1 / 4 + m2
A = np.array(
    [
        [0, 0, 1, 0],
        [0, 0, 0, 1],
        [0, -0.75 * m1 * g / temp, -b / temp, 0],
        [0, 1.5 * mass * g / (temp * ell), 1.5 * b / (temp * ell), 0],
    ]
)
B = np.array([[0, 0, 1 / temp, -1.5 / (temp * ell)]]).T
Cm = np.eye(2, 4)  # measure z and theta
Cr = np.eye(1, 4)  # only command z
D = np.zeros((2, 1))

##### Chapter 8
force_max = 5.0  # max force (N)
theta0 = np.radians(10.0)  # (rad)
