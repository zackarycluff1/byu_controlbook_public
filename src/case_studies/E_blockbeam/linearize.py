import sympy as sp
from sympy import pprint

# Import symbolic objects from generator
from case_studies.E_blockbeam.generate_state_variable_form import (
    state,
    ctrl_input,
    state_dot,
    z,
    theta,
    zd,
    thetad,
    F
)
from case_studies.E_blockbeam.params import (
    g,
    m1,
    m2,
    ell, 
    z_e
)


def linearize_blockbeam():

    # Jacobians


    A = state_dot.jacobian(state)
    B = state_dot.jacobian(ctrl_input)

    A = sp.trigsimp(A.subs([(theta, 0), (z, z_e), (F, g*m2/2 - g*m1*z/ell), ("ell", 0.5), ("m1", m1), ("m2", 2)]))
    B = sp.trigsimp(B.subs([(theta, 0), (z, z_e), (F, g*m2/2 - g*m1*z/ell), ("ell", 0.5), ("m1", m1), ("m2", 2)]))

    A = sp.simplify(A.subs([(theta, 0), (z, z_e), (F, g*m2/2 - g*m1*z/ell), ("ell", 0.5), ("m1", m1), ("m2", 2)]))
    B = sp.simplify(B.subs([(theta, 0), (z, z_e), (F, g*m2/2 - g*m1*z/ell), ("ell", 0.5), ("m1", m1), ("m2", 2)]))


    print("\nState dot:")
    pprint(state_dot)

    print("\nA matrix:")
    pprint(A)

    print("\nB matrix:")
    pprint(B)

    return A, B


if __name__ == "__main__":
    linearize_blockbeam()
