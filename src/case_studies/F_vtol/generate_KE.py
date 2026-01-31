# %%
import sympy as sp
from sympy.physics.vector import dynamicsymbols
from sympy.physics.vector.printing import vlatex
from IPython.display import Math, display

# local (controlbook)
from case_studies.common import sym_utils as su

# This makes it so printing from su only happens when running this file directly
su.enable_printing(__name__ == "__main__")

# %%
####################################################################################
# example from Case Study B to find Kinetic Energy (with vectors/matrices)
####################################################################################
# importing these functions directly to make life a little easier and code a little more readable
from sympy import sin, cos, diff, Matrix, diag, symbols, Function, simplify, latex

# defining mathematical variables (called symbols in sp) and time varying functions like z, theta and h
t, m1, m2, m3, ell, g, d, Jc, m_c, m_r = symbols("t, m1, m2, m3, ell, g, d, Jc, m_c, m_r")

z = dynamicsymbols("z")
theta = dynamicsymbols("theta")
h = dynamicsymbols("h")


# defining generalized coords and derivatives
q = Matrix([[z], [h], [theta]])
qdot = q.diff(t)

# defining the kinetic energy
p1 = Matrix([[z - d * cos(theta)], [h - d * sin(theta)], [0]])
p2 = Matrix([[z + d * cos(theta)], [h + d * sin(theta)], [0]])
p3 = Matrix([[z], [h], [0]])

v1 = diff(p1, t)
v2 = diff(p2, t)
v3 = diff(p3, t)

# by inspection, we can find the angular velocity of the rod
omega = Matrix([[0], [0], [theta.diff(t)]])

# if we are uncomfortable with the above, we can use the rotation matrix describing
# the orientation of the rod relative to the inertial frame to find the angular velocity
R = Matrix([[cos(theta), -sin(theta), 0], [sin(theta), cos(theta), 0], [0, 0, 1]])
#   Omeg_mat = R.diff(t)*R.T
#   omega = Omeg_mat.vee()

# next we define the inertia tensor for the rod
J = diag(0, 0, Jc)


# %%
K = 0.5 * m1 * v1.T @ v1 + 0.5 * m1 * v2.T @ v2 + 0.5 * m3 * v3.T @ v3 + 0.5 * omega.T @ R @ J @ R.T @ omega

display(Math(vlatex(K)))

# mapping = {"m1": "m_r", "m2": "m_r", "m3": "m_c"}
# K=K.subs(mapping)

K = simplify(K)
# K = simplify(
#     0.5 * m1 * v1.T @ v1 + 0.5 * m2 * v2.T @ v2 + 0.5 * m3 * v3.T @ v3 + 0.5 * omega.T @ R @ J @ R.T @ omega
# )

#make dictionary substitutions map m1 to mr m2 to mr and m3 to mc to simplify further

# just grabbing the scalar inside this matrix so that we can do L = K-P, since P is a scalar
K = K[0, 0]


# %%
display(Math(vlatex(K)))

# %%
