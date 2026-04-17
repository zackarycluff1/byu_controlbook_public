"""
Rod-Mass System Parameters

Physical system: Point mass on massless rod with nonlinear spring
"""
import numpy as np
import control as cnt

##### Physical Parameters
m = 0.1              # mass of the point mass (kg)
ell = 0.25           # length of the rod (m)
b = 0.1              # damping coefficient (N⋅m⋅s)
g = 9.8              # gravity (m/s²)
k1 = 0.02            # linear spring coefficient
k2 = 0.01            # cubic spring coefficient (nonlinear term)

# TODO: add any additional parameters needed for simulation or control here ...
