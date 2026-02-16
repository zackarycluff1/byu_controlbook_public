# %%
from case_studies.H_hummingbird.generate_state_variable_form import *
import case_studies.H_hummingbird.params as P

import sympy as sp

# Define accelerations
phiddot, thetaddot, psiddot = sp.symbols("phiddot thetaddot psiddot")
qddot = sp.Matrix([phiddot, thetaddot, psiddot])

# Build full EOM
EOM = M*qddot + C + dP_dq - tau

# Extract pitch equation (row 2)
theta_eq = sp.simplify(EOM[1])

sp.pretty_print(theta_eq)

# %%
theta_eq_longitudinal = theta_eq.subs({phi:0, phidot:0, psidot:0, psiddot:0})

sp.simplify(theta_eq_longitudinal)

theta_long = sp.simplify(theta_eq_longitudinal)

sp.pretty_print(theta_long)

theta_eq_hover = theta_long.subs({thetadot:0, thetaddot:0, theta:0})

theta_eq_hover = sp.simplify(theta_eq_hover)

print("\nEquation at hover:")
sp.pretty_print(theta_eq_hover)


#%%
F = sp.symbols("F")

theta_eq_hover = theta_eq_hover.subs(f_l + f_r, F)

Fe = sp.solve(theta_eq_hover, F)[0]

Fe = sp.simplify(Fe)

print("\nEquilibrium Force Fe:")
sp.pretty_print(Fe)


# %%
theta_long_F = theta_long.subs(f_l + f_r, F)

print("\nLongitudinal equation with F:")
sp.pretty_print(theta_long_F)

theta_ddot_expr = sp.solve(theta_long_F, thetaddot)[0]
theta_ddot_expr = sp.simplify(theta_ddot_expr)

print("\nTheta double dot expression:")
sp.pretty_print(theta_ddot_expr)

# %%
F_control = sp.symbols("F_control")

F_fl = ((m1*ell_1 + m2*ell_2)*g/ell_T) * sp.cos(theta)

F_total = F_fl + F_control

# Substitute back into theta_ddot expression
theta_ddot_linear = theta_ddot_expr.subs(F, F_total)
theta_ddot_linear = sp.simplify(theta_ddot_linear)

print("\nAfter feedback linearization substitution:")
sp.pretty_print(theta_ddot_linear)

# %%
# lateral section

tau, F = sp.symbols("tau, F")


EOM_lat = EOM.subs({
    theta: 0,
    thetadot: 0,
    thetaddot: 0,
    f_l + f_r: F,
    d*(f_l - f_r): tau,
    P.beta: 0
})


phi_eq = sp.simplify(EOM_lat[0])
psi_eq = sp.simplify(EOM_lat[2])

print("\nRoll equation:")
sp.pretty_print(phi_eq)

print("\nYaw equation:")
sp.pretty_print(psi_eq)


phi_eq = phi_eq.subs(F, Fe)
psi_eq = psi_eq.subs(F, Fe)

phi_eq = sp.simplify(phi_eq)
psi_eq = sp.simplify(psi_eq)

# %%
phi_e, tau_e = sp.symbols("phi_e tau_e")

eq_phi = phi_eq.subs({
    phidot: 0,
    psidot: 0,
    phiddot: 0,
    psiddot: 0,
    phi: phi_e,
    tau: tau_e
})

eq_psi = psi_eq.subs({
    phidot: 0,
    psidot: 0,
    phiddot: 0,
    psiddot: 0,
    phi: phi_e,
    tau: tau_e
})

# Solve equilibrium conditions
sol = sp.solve([eq_phi, eq_psi], [phi_e, tau_e])

print("\nEquilibrium solution:")
sp.pretty_print(sol)

phiddot_expr = sp.solve(phi_eq, phiddot)[0]
psiddot_expr = sp.solve(psi_eq, psiddot)[0]

# Define state vector
x = sp.Matrix([psi, phi, psidot, phidot])
u = sp.Matrix([tau])

# State equations
xdot = sp.Matrix([
    psidot,
    phidot,
    psiddot_expr,
    phiddot_expr
])

# %%
A = xdot.jacobian(x)
B = xdot.jacobian(u)

# Substitute equilibrium
A = A.subs({
    phi: 0,
    phidot: 0,
    psidot: 0,
    tau: 0
})

B = B.subs({
    phi: 0,
    phidot: 0,
    psidot: 0,
    tau: 0
})

print("\nA matrix:")
sp.pretty_print(sp.simplify(A))

print("\nB matrix:")
sp.pretty_print(sp.simplify(B))