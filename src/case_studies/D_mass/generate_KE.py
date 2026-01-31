# %%
import sympy as sp
from sympy.physics.vector import dynamicsymbols
from sympy.physics.vector.printing import vlatex
from IPython.display import Math, display

####################################################################################
# example from Case Study A to find Kinetic Energy (with vectors/matrices)
####################################################################################
# importing these functions directly to make life a little easier and code a little more readable
from sympy import sin, cos, Matrix, symbols, simplify, diff

# init_printing(use_latex=True)

# defining mathematical variables (called symbols in sp) and time varying functions like z and theta
t, m, ell = symbols("t, m, ell")

# defining generalized coord and their derivatives
z = dynamicsymbols("z")
q = Matrix([[z]])
qdot = q.diff(t)

# to find KE, start with the position of the mass in the inertial frame, then find the velocity
p = Matrix([[z], [0], [0]])
v = p.diff(t)

# define the rotation matrix describing the orientation of the rod relative to the inertial frame
# R = Matrix([[cos(theta), -sin(theta), 0], [sin(theta), cos(theta), 0], [0, 0, 1]])

# find the angular velocity of the rod
# Omeg_mat = simplify(R.diff(t) * R.T)

# omega = Matrix([[Omeg_mat[2, 1]], [Omeg_mat[0, 2]], [Omeg_mat[1, 0]]])
# the following function does the same thing as above
# (extracting omega - vector - from Omeg_mat - a skew symmetric matrix)
#   omega = Omeg_mat.vee()

# next we define the inertia tensor for the rod
#J = sp.diag(0, 1.0 / 12 * m * ell**2, 1.0 / 12 * m * ell**2)

# calculate the kinetic energy and display it
K = simplify(0.5 * m * v.T @ v) #+ 0.5 * omega.T @ R @ J @ R.T @ omega)
K = K[0, 0]

display(Math(vlatex(K)))
