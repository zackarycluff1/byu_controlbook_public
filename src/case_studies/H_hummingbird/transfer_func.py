# %%

import sympy as sp
from sympy.physics.vector import dynamicsymbols
from sympy.physics.vector.printing import vlatex
from IPython.display import Math, display
import numpy as np
import case_studies.H_hummingbird.generate_linearization as line

from sympy import sin, cos, Matrix, symbols, simplify, diff, solvers, linsolve

Jc, u, mc, mr, s, g, ell, Fe, d = symbols("J_c, u, m_c, m_r, s, g, ell, F_e, d")


C1 = Matrix([[1, 0, 0, 0]])



A2 = Matrix([[0, 0, 1, 0], [0, 0, 0, 1], [0, -Fe/(mc+2*mr), -u/(mc+2*mr), 0], [0, 0, 0, 0]])

B2 = Matrix([[0], [0], [0], [1/(Jc+2*mr*d**2)]])

C2 = Matrix([[1, 0, 0, 0], [0, 1, 0, 0]])



I1 = np.eye(4)

I2 = np.eye(4)

# L = C(sI-A)^-1+D

H1 = C1 @ (s * I1 - line.A_lat).inv() @ line.B_lat

H2 = C2 @ (s * I2 - A2).inv() @ B2

H1 = H1.subs([(mc, 1), (mr, 0.5)])

H2 = H2.subs([(mc, 1), (mr, 0.5), (u, 0.1), (d, 0.3), (Jc, 0.0042)])

H1 = sp.simplify(H1)

H2 = sp.simplify(H2)

display(Math(vlatex((sp.cancel(H1)))))

display(Math(vlatex((sp.cancel(H2)))))

# H = sp.simplify(H)

# display(Math(vlatex((sp.cancel(H)))))

# display(Math(vlatex(H)))
# %%
