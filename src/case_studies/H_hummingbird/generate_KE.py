# %%
################################################################################
# This file is meant to be run interactively with VSCode's Jupyter extension.
# It works as a regular Python script as well, but the printed results will not
# display as nicely.
################################################################################

# NOTE: different versions of Python/sympy can lead to different simplifications.
# The simplifications shown here were designed with Python 3.13.7 with sympy 1.14.0

# %%
# 3rd-party
import sympy as sp
from IPython.display import display

# local (controlbook)
# these functions are defined in our helper functions file in the public repository
from case_studies.common.sym_utils import rotx, roty, rotz, calc_omega, find_coeffs, enable_printing, dynamicsymbols, printeq
from sympy import sin, cos, Matrix, symbols, simplify, diff

# This makes it so printing from su only happens when running this file directly
enable_printing(__name__ == "__main__")

# %% [markdown]
# # Lab H.2: Calculate Kinetic Energy
# %%

# Defining necessary symbols and variables to use in the calculations e.g. t, ell_1, m1, J1x, phi, theta, psi, etc.
# TODO define all necessary symbols
t, ell_1, ell_2, ell_3x, ell_3y, ell_3z, m1, m2, m3, J1x, J1y, J1z, J2x, J2y, J2z, J3x, J3y, J3z, d = symbols("t, ell_1, ell_2, ell_3x, ell_3y, ell_3z, m1, m2, m3, J_1x, J_1y, J_1z, J_2x, J_2y, J_2z, J_3x, J_3y, J_3z, d")

# TODO Define time-varying symbols for generalized coordinates and their derivatives
phi, theta, psi = dynamicsymbols("phi, theta, psi")

q = Matrix([phi, theta, psi])
qdot = q.diff(t)

# %% [markdown]
# ### Linear Velocity Terms
# %%
# TODO define the position of each mass in body frame, then rotate
# it into the world or inertial frame.
p1_in_b = Matrix([ell_1, 0, 0])
# p1_in_w = Matrix([d, 0, 0])
p1_in_w = rotz(psi) * roty(theta) * rotx(phi) * p1_in_b

p2_in_2 = Matrix([ell_2, 0, 0])
# p2_in_w = Matrix([ell_2 * cos(theta), 0, -ell_2 * sin(theta)])
p2_in_w = rotz(psi) * roty(theta) * p2_in_2

p3_in_1 = Matrix([ell_3x, ell_3y, ell_3z])
# p3_in_w = Matrix([0, 0, -ell_1])
p3_in_w = rotz(psi) * p3_in_1


# %%
# TODO: take the time derivative of the position vectors to get the linear velocity,
# then use the "find_coeffs" function to calculate the "V_i" matrices

v1_in_w = p1_in_w.diff(t)
V1 = find_coeffs(v1_in_w, qdot)

v2_in_w = p2_in_w.diff(t)
V2 = find_coeffs(v2_in_w, qdot)

v3_in_w = p3_in_w.diff(t)
V3 = find_coeffs(v3_in_w, qdot)

# printeq("v_1", v1_in_w)
# printeq("v_2", v2_in_w)
# printeq("v_3", v3_in_w)

# printeq("V_1", V1)
# printeq("V_2", V2)
# printeq("V_3", V3)



# now calculate the rotation matrices for each rigid body
# TODO use the "rotx", "roty", and "rotz" functions to calculate the rotation matrices
# for each rigid body
R1 = rotz(psi) * roty(theta) * rotx(phi) #rotation to body 1
R2 = rotz(psi) * roty(theta) #rotation to body 2
R3 = rotz(psi) #rotation to body 3



# we can use the rotation matrices to calculate the angular velocity of each rigid body
# TODO use the "calc_omega" function to calculate the angular velocity of each rigid body

omega_1 = calc_omega(R1)
omega_2 = calc_omega(R2)
omega_3 = calc_omega(R3)

# Simplify the angular velocities
omega_1 = sp.simplify(omega_1)
omega_2 = sp.simplify(omega_2)
omega_3 = sp.simplify(omega_3)


# TODO use the "find_coeffs" function to calculate the "W_i" matrices
W1 = find_coeffs(omega_1, qdot) 
W2 = find_coeffs(omega_2, qdot)
W3 = find_coeffs(omega_3, qdot)

# printeq("omega_1", omega_1)
# printeq("omega_2", omega_2)
# printeq("omega_3", omega_3)

# printeq("W_1", W1)
# printeq("W_2", W2)
# printeq("W_3", W3)

# %% [markdown]
# ### Inertia Tensor Terms
# %%
# TODO: create the diagonal inertia tensors for each rigid body using inertia symbols

J1 = Matrix([[J1x, 0, 0], [0, J1y, 0], [0, 0, J1z]]) 
J2 = Matrix([[J2x, 0, 0], [0, J2y, 0], [0, 0, J2z]])
J3 = Matrix([[J3x, 0, 0], [0, J3y, 0], [0, 0, J3z]]) 


# %% [markdown]
# ### Generalized Mass/Inertia Matrix
# %%
# TODO: calculate M using the masses and the V, W, R, and J matrices
M = sp.zeros(3, 3)
M = M + m1 * (V1.T @ V1) + (W1.T @ R1 @ J1 @ R1.T @ W1)
M = M + m2 * (V2.T @ V2) + (W2.T @ R2 @ J2 @ R2.T @ W2)
M = M + m3 * (V3.T @ V3) + (W3.T @ R3 @ J3 @ R3.T @ W3)


# Simplify
M = sp.trigsimp(M)  # because there seemed to be many trig terms that could simplify
sp.pprint(M)
display(M)
# NOTE: to match the current lab manual, do not use any further simplifications.

# %%
# Display the result (separate long terms to match lab manual) - this is just to help you 
# compare your result to the lab manual. 
M22 = M[1, 1]
M23 = M[1, 2]
M33 = M[2, 2]
M22_sym = sp.Symbol("M_22")
M23_sym = sp.Symbol("M_23")
M33_sym = sp.Symbol("M_33")
long_terms = {
    M22: sp.Symbol("M_22"),
    M23: sp.Symbol("M_23"),
    M33: sp.Symbol("M_33"),
}
printeq("M", M.subs(long_terms))

# Because there were 2 terms with "J_1y", collect it and trigsimp to condense to 1
M22 = sp.collect(M22, M22.free_symbols)
M22 = sp.trigsimp(M22)
printeq("M_22", M22)

# No noticeable simplifications here, just print it
printeq("M_23", M23)

# Because some inertia terms were repeated twice, collect them and trigsimp to condense
M33 = sp.collect(M33, M33.free_symbols)
M33 = sp.trigsimp(M33)
# Because multiple terms had sin(theta)**2 and cos(theta)**2, collect those too
M33 = sp.collect(M33, [sp.sin(theta) ** 2, sp.cos(theta) ** 2])  # type: ignore
printeq("M_33", M33)

# %% [markdown]
# # Calculate Kinetic Energy
# None of the following is required by the lab, but it helps you see how we 
# we can formulate M, and use it to calculate Kinetic Energy.
# %%
# Define 1/2 as a rational number to avoid floating point issues
half = sp.Rational(1, 2)
K = half * qdot.T @ M @ qdot  # calculate the kinetic energy
K = K[0, 0]  # make K a scalar instead of a 1x1 matrix

# substitute long terms for readability
printeq("K", sp.simplify(K.subs(long_terms)))
