#!/usr/bin/env python3
import sympy as sp
from forward_kinematic import FORWARD_KINEMATIC


L1, L2 = sp.symbols('L1 L2')
q1, q2 = sp.symbols('q1 q2')

# Define angular axes ω
s1 = [0, 0, 1]
s2 = [0, 0, 1]

# Define joint positions q (points on axes)
q1_point = [0, 0, 0]
q2_point = [-L1, 0, 0]

# Home configuration
M = sp.Matrix([
    [1, 0, 0, -(L1 + L2)],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

FK = FORWARD_KINEMATIC([s1, s2], [q1_point, q2_point], M, [q1, q2], [L1, L2])
T_numeric = FK.Forward_kinematic_result((1.57, 1.57), (1,1))
print(T_numeric[0:3,3])  # Position x, y, z

# Compare with standard FK
print(FK.standard_FK_2_dof_result((1.57, 1.57), (1,1)))
