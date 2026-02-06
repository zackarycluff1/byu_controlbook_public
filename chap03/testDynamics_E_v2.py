# run this file to test your dynamics file
import numpy as np
from case_studies.E_blockbeam.dynamics import BlockbeamDynamics
# x = [z, theta, zdot, thetadot]
x_tests = np.array([
    [0.0, 0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0, 1.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0, 1.0],
    [-1.0, -10.0, 0.0, 1.0],
    [0.0, 5.0, -5.0, -5.0],
    [1.0, 2.0, 3.0, 4.0],
    [-1.0, 2.0, -3.0, 4.0],
    [1.0, 1.0, 1.0, 1.0],
], dtype=np.float64).reshape(-1, 4, 1)

# u = F
u_tests = np.array([
0.0, 
10.0, 
100.0, 
5.0, 
50.0, 
-10.0, 
-1.0, 
-20.0, 
-50.0, 
50.0
], dtype=np.float64).reshape(-1, 1)

xdot_tests = np.array([
    [0.0, 0.0, 0.0, -29.4],
    [0.0, 1.0, 1.0, -6.445161290322581],
    [0.0, 0.0, -8.246415651117387, 146.20580396791863],
    [1.0, 0.0, 0.0, -14.399999999999999],
    [0.0, 1.0, 1.0, 32.26451612903226],
    [0.0, 1.0, -6.331406886715824, 10.507340889918673],
    [-5.0, -5.0, 9.397457891698757, -9.19065480900853],
    [3.0, 4.0, 7.088885217108318, -1.4942486827565409],
    [-3.0, 4.0, -24.91111478289168, 5.062077606586185],
    [1.0, 1.0, -7.2464156511173865, 16.077753752558497],
], dtype=np.float64).reshape(-1, 4, 1)

num_tests = len(x_tests)
num_passed = 0
system = BlockbeamDynamics()
xdot_eqns = np.array(['zd', 'thetad', 'zdd', 'thetadd'])
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
