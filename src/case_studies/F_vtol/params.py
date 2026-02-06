import numpy as np

##### Chapter 2
# Physical parameters
Jc = 0.0042 # (kg)
k = 0.1  # spring constant (Nm)
b = 0.1  # damping coefficient (Nms)
d = 0.3
g = 9.81
m1 = mr = 0.25
m2 = mr = 0.25
m3 = mc = 1

##### Chapter 3
# Initial Conditions
h0 = 0
theta0 = np.radians(0.0)  # (rad)
z0 = 0.0  # (rad)
thetadot0 = 0.0  # (rad/s)
zdot0 = 0.0  # (rad/s)
hdot0 = 0.0

# Simulation parameters
t0 = 0.0  # start time
tf = 50.0  # end time
ts = 0.01  # integration time step

mixing = np.array([
    [0.5,  1/(2*d)],   # Fr
    [0.5, -1/(2*d)],   # Fl
])

unmixer = np.array([
    [1.0, 1.0],        # F
    [d,  -d],          # tau = d*(Fr - Fl)
])


##### Chapter 4
# Linearization/equilibrium point
x_eq = np.zeros(4)
u_eq = np.array([0.0])

##### Chapter 5
# Transfer function numerator and denominator
# inertia = Js + Jp
# tf_inner_num = [1 / inertia]
# tf_inner_den = [1, 0, 0]
# tf_outer_num = [b / Jp, k / Jp]
# tf_outer_den = [1, b / Jp, k / Jp]

##### Chapter 6 / 11-14
# State space
# A = np.array(
#     [
#         [0, 0, 1, 0],
#         [0, 0, 0, 1],
#         [-k / Js, k / Js, -b / Js, b / Js],
#         [k / Jp, -k / Jp, b / Jp, -b / Jp],
#     ]
# )
# B = np.array([[0, 0, 1 / Js, 0]]).T
# Cm = np.eye(2, 4)  # measure theta and phi
# from section 6.4, Cm = np.array([[1, 0, 0, 0],
#                                  [-1, 1, 0, 0]])
# because we say that we can only measure theta and
# the relative angle (phi - theta)

# Cr = Cm[[1]]  # only command phi
D = np.zeros((2, 1))

##### Chapter 8
force_max = 5000.0  # max torque (Nm)