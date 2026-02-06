import sympy as sp
from sympy import pprint, symbols
from case_studies.common import sym_utils as su
# Import symbolic objects from generator
from case_studies.F_vtol.generate_state_variable_form import (
    state,
    ctrl_input,
    state_dot,
    z,
    theta,
    zd,
    thetad,
    F,
    tau
)
from case_studies.F_vtol.params import (
    g,
    m1,
    m2,
    m3,
    d,
    Jc, 
    b
)


def linearize_vtol():

    
    # Jacobians

    A = state_dot.jacobian(state)
    B = state_dot.jacobian(ctrl_input)

    A = sp.trigsimp(A.subs([(theta, 0), (tau, 0.0), (F, g * (m3 + 2 * m1)), ("Fr", 0.5 * F + 0.5 * tau / d), ("Fl", 0.5 * F - 0.5 * tau / d)]))
    B = sp.trigsimp(B.subs([(theta, 0), (tau, 0.0), (F, g * (m3 + 2 * m1)), ("Fr", 0.5 * F + 0.5 * tau / d), ("Fl", 0.5 * F - 0.5 * tau / d)]))

    A = sp.simplify(A.subs([(theta, 0), (tau, 0.0), (F, g * (m3 + 2 * m1)), ("Fr", 0.5 * F + 0.5 * tau / d), ("Fl", 0.5 * F - 0.5 * tau / d)]))
    B = sp.simplify(B.subs([(theta, 0), (tau, 0.0), (F, g * (m3 + 2 * m1)), ("Fr", 0.5 * F + 0.5 * tau / d), ("Fl", 0.5 * F - 0.5 * tau / d)]))


    print("\nState dot:")
    pprint(state_dot)

    print("\nA matrix:")
    pprint(A)

    print("\nB matrix:")
    pprint(B)

    return A, B


if __name__ == "__main__":
    linearize_vtol()
