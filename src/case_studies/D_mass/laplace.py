# %%

import sympy as sp
from sympy.physics.vector import dynamicsymbols
from sympy.physics.vector.printing import vlatex
from IPython.display import Math, display
import numpy as np

from sympy import sin, cos, Matrix, symbols, simplify, diff, solvers, linsolve

k, b, m, s = symbols("k, b, m, s")

A = Matrix([[0, 1], [-k/m, -b/m]])

C = Matrix([[1, 0]])

B = Matrix([[0], [1/m]])

I = np.eye(2)

# L = C(sI-A)^-1+D

H = C @ (s * I - A).inv() @ B


H = sp.simplify(H)
display(Math(vlatex((sp.cancel(H[0, 0])))))

# display(Math(vlatex(H)))
# %%
