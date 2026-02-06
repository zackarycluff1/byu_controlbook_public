import numpy as np

ts = 0.01
g = 9.81 
ell1 = 0.247
ell2 = 0.039
ell3x = 0.007
ell3y = -0.007
ell3z = 0.018
ellT = 0.355
d = 0.12
m1 = 0.108862
J1x = 0.000189
J1y = 0.001953
J1z = 0.001894
m2 = 0.4717
J2x = 0.00231
J2y = 0.003274
J2z = 0.003416
m3 = 0.1905
J3x = 0.0002222
J3y = 0.0001956
J3z = 0.000027


##### Chapter 4
# mixing matrices (see end of Chapter 4 in lab manual)
# mixing is a UAV term for taking body forces/torques to individual motor forces
unmixer = np.array([[1.0, 1.0], [d, -d]])  # [F, tau] = unmixer @ [fl, fr]
mixer = np.linalg.inv(unmixer)  # [fl, fr] = mixer @ [F, tau]

