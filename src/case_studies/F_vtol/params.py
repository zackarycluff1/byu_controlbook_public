import numpy as np

##### Chapter 2
# Physical parameters
Jc = 0.0042 # (kg)
k = 0.1  # spring constant (Nm)
b = 0.1  # damping coefficient (Nms)
d = 0.3
g = 9.8
m1 = mr = 0.25
m2 = mr = 0.25
m3 = mc = 1

m_total = m1 + m2 + m3
F_eq = m_total * g

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
x_eq_lat = np.zeros(4)
x_eq_lon = np.zeros(2)
u_eq = np.array([g*(mc+2*mr), 0.0])

##### Chapter 5

tf_altitude_num = [1 / (mc+2*mr)]
tf_altitude_den = [1, 0, 0]

tf_theta_num = [1 / (Jc+2*mr*d**2)]
tf_theta_den = [1, 0, 0]

tf_z_num = [-u_eq[0] / (mc+2*mr)]
tf_z_den = [1, b / (mc+2*mr), 0]
##### Chapter 6 / 11-14
# State space
A_lon = np.array(
    [
        [0, 1],
        [0, 0],
    ]
)
B_lon = np.array([[0, 1/(2*mr+mc)]]).T
Cm_lon = np.array([[1,0]])

A_lat = np.array(
    [
        [0, 0, 1, 0],
        [0, 0, 0, 1],
        [0, (-F_eq/(mc+2*mr)), (-b/(mc+2*mr)), 0],
        [0, 0, 0, 0],
    ]
)
B_lat = np.array([[0, 0, 0, 1/(Jc+2*mr*d**2)]]).T
Cm_lat = np.eye(2, 4)  # measure z and theta (used for controller design)
Cm_lat_obs = np.array([[1, 0, 0, 0]])  # z only — what Dynamics.h() actually provides for lateral
Cr = np.array([[1,0,0,0]])   # command z
D = np.zeros((2, 1))

##### Chapter 8
force_max = 5000.0  # max torque (Nm)