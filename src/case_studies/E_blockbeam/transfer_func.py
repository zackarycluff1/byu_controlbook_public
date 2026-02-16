# %%

import sympy as sp
from sympy.physics.vector import dynamicsymbols
from sympy.physics.vector.printing import vlatex
from IPython.display import Math, display
import numpy as np

from sympy import sin, cos, Matrix, symbols, simplify, diff, solvers, linsolve

k, b, m1, m2, s, g, ell, ze = symbols("k, b, m_1, m_2, s, g, ell, z_e")

A = Matrix([[0, 0, 1, 0], [0, 0, 0, 1], [0, -g, 0, 0], [(-m1*g)/((m2*ell**2)/3 + m1*ze**2), 0, 0, 0]])
#A = Matrix([[0, 0, 1, 0], [0, 0, 0, 1], [0, -g, 0, 0], [0, 0, 0, 0]])

C = Matrix([[1, 0, 0, 0], [0, 1, 0, 0]])

B = Matrix([[0], [0], [0], [ell/(m2*ell**2/3 + m1*ze**2)]])

I = np.eye(4)

# L = C(sI-A)^-1+D

H = C @ (s * I - A).inv() @ B

H = sp.simplify(H)

display(Math(vlatex((sp.cancel(H)))))

H = H.subs([(m1, 0.35), (m2, 2), (ell, 0.5), (ze, 0.25)])

H = sp.simplify(H)

# display(Math(vlatex((sp.cancel(H)))))

# display(Math(vlatex(H)))
# %%
