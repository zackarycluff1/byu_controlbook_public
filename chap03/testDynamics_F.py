# run this file to test your dynamics file
import numpy as np
from case_studies.F_vtol.dynamics import VtolDynamics
# x = [z, h, theta, zdot, hdot, thetadot]
x_tests = np.array([
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    [0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    [0.0, 0.0, 1.0, 0.0, -1.0, -1.0],
    [1.0, 0.0, 0.0, 1.0, 0.0, -10.0],
    [-1.0, -10.0, 0.0, 1.0, 50.0, 5.0],
    [0.0, 5.0, -5.0, -5.0, 5.0, -50.0],
    [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
    [-1.0, 2.0, -3.0, 4.0, -5.0, 6.0],
    [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
], dtype=np.float64).reshape(-1, 6, 1)

# u = [fr, fl]
u_tests = np.array([
    [0.0, 0.0],
    [1.0, 1.0],
    [-100.0, 100.0],
    [2.0, 5.0],
    [-2.0, -5.0],
    [0.1, 0.5],
    [-1.5, -0.9],
    [3.0, -20.0],
    [5.0, 5.0],
    [10.0, 8.0],
], dtype=np.float64).reshape(-1, 2)

xdot_tests = np.array([
    [0.0, 0.0, 0.0, 0.0, -9.81, 0.0],
    [0.0, 1.0, 0.0, 0.0, -8.476666666666667, 0.0],
    [0.0, 0.0, 1.0, 0.0, -9.81, -1219.5121951219512],
    [0.0, -1.0, -1.0, -3.9268645957701835, -7.288589239282015, -18.292682926829265],
    [1.0, 0.0, -10.0, -0.06666666666666667, -14.476666666666667, 18.292682926829265],
    [1.0, 50.0, 5.0, -0.06666666666666667, -9.41, -2.4390243902439024],
    [-5.0, 5.0, -50.0, 1.8676121727943549, -10.263859496741162, -3.6585365853658534],
    [4.0, 5.0, 6.0, 1.3326934246784952, 1.409914961471716, 140.24390243902437],
    [4.0, -5.0, 6.0, 0.6741333870657815, -16.4099499773363, 0.0],
    [1.0, 1.0, 1.0, -10.164318484361425, -3.326372329582322, 12.195121951219512],
], dtype=np.float64).reshape(-1, 6, 1)

num_tests = len(x_tests)
num_passed = 0
system = VtolDynamics()
xdot_eqns = np.array(['zd', 'hd', 'thetad', 'zdd', 'hdd', 'thetadd'])
precision = 6

for i, (x, u, xdot_expected) in enumerate(zip(x_tests, u_tests, xdot_tests), start=1):
    xdot_actual = system.f(x, u)
    xdot_expected_flat = xdot_expected.flatten()
    error = xdot_actual - xdot_expected_flat
    correct_indices = np.abs(error) < 1e-12
    if correct_indices.all():
        print(f"Test {i:>2}: PASS")
        num_passed += 1
    else:
        incorrect_indices = np.where(~correct_indices)[0]
        print(f"Test {i:>2}: FAIL - check equations for {xdot_eqns[incorrect_indices]}")
        for idx in incorrect_indices:
            print(f'{xdot_eqns[idx]:>20}: ', end='')
            print(f'yours = {xdot_actual[idx]:<{precision+8}.{precision}g}', end='')
            print(f'expected = {xdot_expected_flat[idx]:.{precision}g}')

if num_passed == num_tests:
    print("\nExcellent work!! The f(x,u) function from your dynamics file has passed all of the tests!\n")
else:
    print(f"\nDarn, {num_passed}/{num_tests} of the tests passed... Keep it up! You'll get it!\n")
