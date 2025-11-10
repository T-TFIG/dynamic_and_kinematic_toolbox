#!/usr/bin/env python3
import sympy as sp


# PARENT CLASS

class TOOLBOX:
    def __init__(self, screw_order, joint_positions, home_configuration, degree_order, link_order):
        """
        screw_order: list of angular axes ω (3x1)
        joint_positions: list of points on the joint axes q (3x1)
        home_configuration: 4x4 symbolic home matrix
        degree_order: list of joint symbols
        link_order: list of link lengths symbols
        """
        self.Screw = screw_order
        self.q_points = joint_positions
        self.M = home_configuration
        self.q = degree_order
        self.L = link_order
        self.FK_symbol = None

        # Compute screw axes on initialization
        self.define_screw_axis()
    
    def define_screw_axis(self):
        """Compute 6x1 screw axes from ω and q_point."""
        new_screws = []
        for i in range(len(self.Screw)):
            omega = sp.Matrix(self.Screw[i])
            q_point = sp.Matrix(self.q_points[i])
            v = -omega.cross(q_point)
            S = sp.Matrix.vstack(omega, v)
            new_screws.append(S)
        self.Screw = new_screws

    def skew_3(self, w):
        """Return skew-symmetric matrix of 3x1 vector w"""
        return sp.Matrix([
            [0, -w[2],  w[1]],
            [w[2],  0, -w[0]],
            [-w[1], w[0], 0]
        ])

    def twist_to_matrix(self, S):
        """Convert 6x1 screw axis to 4x4 matrix form"""
        w = S[0:3, 0]
        v = S[3:6, 0]
        return sp.Matrix.vstack(
            sp.Matrix.hstack(self.skew_3(w), v),
            sp.Matrix([[0, 0, 0, 0]])
        )


# CHILD CLASS (FK)

class FORWARD_KINEMATIC(TOOLBOX):
    def __init__(self, toolbox: TOOLBOX):
        # Copy all TOOLBOX attributes to this class
        self.toolbox = toolbox # just get all the dictionary inside the parent class to your class

    def Forward_kinematic_formula(self):
        """Compute symbolic forward kinematic transformation"""
        T = sp.eye(4)
        for i in range(len(self.toolbox.Screw)):
            S = self.toolbox.Screw[i]
            q_i = self.toolbox.q[i]
            T = T @ sp.exp(self.toolbox.twist_to_matrix(S) * q_i)
        T = T @ self.toolbox.M
        self.toolbox.FK_symbol = sp.simplify(T)
        return sp.simplify(T)

    def Forward_kinematic_result(self, joint_values, link_values):
        """Compute numeric forward kinematics"""
        T_sym = self.Forward_kinematic_formula()
        sub_joint = {self.toolbox.q[i]: joint_values[i] for i in range(len(self.toolbox.q))}
        sub_link = {self.toolbox.L[i]: link_values[i] for i in range(len(self.toolbox.L))}
        sub_dict = sub_joint | sub_link
        T_numeric = T_sym.evalf(subs=sub_dict, chop=True)
        return T_numeric

    def standard_FK_2_dof_result(self, joint_value, link_value):
        """Closed-form planar FK for 2-DOF"""
        q1, q2 = joint_value
        L1, L2 = link_value
        x = L1 * sp.sin(q1) + L2 * sp.sin(q1 + q2)
        y = -L1 * sp.cos(q1) - L2 * sp.cos(q1 + q2)
        return float(x.evalf()), float(y.evalf())

    def print_the_formula(self):
        sp.pretty_print(self.FK_symbol)

# CHILD CLASS (IK)
    
class INVERSE_KINEMATIC(TOOLBOX):
    def __init__(self, toolbox: TOOLBOX):
        self.toolbox = toolbox
    

    

    



