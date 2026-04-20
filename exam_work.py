# %%
import sympy as sp
from sympy.physics.vector import dynamicsymbols
from sympy.physics.vector.printing import vlatex
from IPython.display import Math, display
import numpy as np
from sympy import sin, cos, Matrix, symbols, simplify, diff, solvers, linsolve

k, b, m, s, ell, theta = symbols("k, b, m, s, ell, theta")
A = Matrix([[0.4, 0.6], [1, -0.2]])
C = Matrix([[1, 0]])
B = Matrix([[0.5], [0]])
I = np.eye(2) # L = C(sI-A)^-1+D
H = C @ (s * I - A).inv() @ B
H = sp.simplify(H)
display(Math(vlatex((sp.cancel(H[0, 0])))))
# %%
p1 = ell * cos(theta)

D = Matrix([[s-1, -2, 4, 8], [-1, s, 0, 0], [0, -1, s, 0], [0, 0, -1, s]])
result = D.det()

D1 = Matrix([[s-3, -1, 0], [0, s, -1], [6, 0, s]])
result1 = D1.det()

C1 = Matrix([[1, -1.5, 1], [0.5, -0.75, 0.5], [2, -3, -3]])
result2 = C1.det()

A1 = Matrix([[s+1, -4, 0], [-1, s, 0], [-1, 1, s]])
result3 = A1.det()

alpha = Matrix([1, 1, 0])

a = Matrix([1, -4, 0])
A11 = Matrix([[1, 1, -4], [0, 1, 1], [0, 0, 1]])
Ctr = Matrix([[1, -1, 5], [0, 1, -1], [0, 1, -2]])

#

A5 = np.array([[-1, 4], [1, 0]])
B5 = np.array([[1], [0]])
K5 = np.array([[2, 5]])
eigenvalues, eigenvectors = np.linalg.eig(A5-B5@K5)

C_r = np.array([[1, -1]])
k_r = 1 / (C_r @ np.linalg.inv(A5-B5@K5)@B5)
display(k_r)