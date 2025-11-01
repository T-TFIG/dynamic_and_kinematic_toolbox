#!/usr/bin/env python3
import sympy as sp

class FORWARD_KINEMATIC:
    def __init__(self, screw_order, joint_positions, home_configuration, degree_order, link_order):
        """
        screw_order: list of angular axes ω (3x1)
        joint_positions: list of points on the joint axes q (3x1)
        home_configuration: 4x4 symbolic home matrix
        degree_order: list of joint symbols
        link_order: list of link lengths symbols
        """
        self.Screw = screw_order          # list of ω (3x1)
        self.q_points = joint_positions   # list of points q (3x1)
        self.M = home_configuration       # 4x4 home matrix
        self.q = degree_order             # list of joint variables
        self.L = link_order               # link lengths symbols
        self.T_screw = sp.eye(4)

    def define_screw_axis(self):
        """Compute 6x1 screw axes from ω and q_point."""
        new_screws = []
        for i in range(len(self.Screw)):
            omega = sp.Matrix(self.Screw[i])
            q_point = sp.Matrix(self.q_points[i])
            v = -omega.cross(q_point)       # linear part
            S = sp.Matrix.vstack(omega, v)  # 6x1 screw
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

    def Forward_kinematic_formula(self):
        """Compute symbolic forward kinematic transformation"""
        T = sp.eye(4)
        self.define_screw_axis()
        for i in range(len(self.Screw)):
            S = self.Screw[i]
            q_i = self.q[i]
            T = T @ sp.exp(self.twist_to_matrix(S) * q_i)
            
        T = T @ self.M
        return sp.simplify(T)

    def Forward_kinematic_result(self, joint_values, link_values):
        """Compute numeric forward kinematics"""
        T_sym = self.Forward_kinematic_formula()
        sub_joint = {self.q[i]: joint_values[i] for i in range(len(self.q))}
        sub_link = {self.L[i]: link_values[i] for i in range(len(self.L))}
        sub_dict = sub_joint | sub_link
        T_numeric = T_sym.evalf(subs=sub_dict, chop=True)
        return T_numeric

    def standard_FK_2_dof_result(self, joint_value, link_value):
        """Closed-form planar FK for 2-DOF"""
        q1, q2 = joint_value
        L1, L2 = link_value
        x = -L1*sp.cos(q1) - L2*sp.cos(q1 + q2)
        y = -L1*sp.sin(q1) - L2*sp.sin(q1 + q2)
        return float(x.evalf()), float(y.evalf())
