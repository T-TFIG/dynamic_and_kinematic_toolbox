#!/usr/bin/env python3
import sympy as sp
from forward_kinematic import TOOLBOX, FORWARD_KINEMATIC

# Define symbols
L1, L2 = sp.symbols('L1 L2')
q1, q2 = sp.symbols('q1 q2')

# Define angular axes ω
s1 = [0, 0, 1]
s2 = [0, 0, 1]

# Define joint positions (points on axes)
q1_point = [0, 0, 0]
q2_point = [0, -L1, 0]

# Define home configuration
M = sp.Matrix([
    [1, 0, 0, 0],
    [0, 1, 0, -(L1 + L2)],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

# Initialize TOOLBOX and FORWARD KINEMATIC
TOOL = TOOLBOX([s1, s2], [q1_point, q2_point], M, [q1, q2], [L1, L2])
FK = FORWARD_KINEMATIC(TOOL)

# Compute numeric forward kinematics
T_numeric = FK.Forward_kinematic_result((1.57, 1.57), (1, 1))
print("End-effector position (x, y, z):")
print(T_numeric[0:3, 3])

# Compare with standard closed-form 2-DOF FK
print("\nClosed-form FK result (x, y):")
print(FK.standard_FK_2_dof_result((1.57, 1.57), (1, 1)))

sp.pretty_print(TOOL.FK_symbol)
# FK.print_the_formula()
