import numpy as np
import sympy as sp

from sympy.physics.vector.printing import vlatex
from IPython.display import Math, display

s = sp.symbols("s")

A = sp.Matrix([[0, 1], [-3.2, 16]])
B = sp.Matrix([[0], [160]])
C = sp.Matrix([[1, 0]])

I = sp.eye(2)

transfer_func = C @ ((s * I) - A).inv() @ B

display(transfer_func)