"""Foot step planner for MORS quadruped robot"""

import numpy as np


class FootStepPlanner():
    """ The foot step planner class""" 

    def __init__(self):
        self.p0_b = np.array([0.106, -0.067, 0.0], dtype=np.float64)
        self.g = 9.81
        self.h = 0.22
        self.k1 = 0.03
        self.k2 = 0.17

    def set_robot_params(self, p0_b):
        """p0_b - shoulder position"""
        self.p0_b = p0_b

    def set_coefficients(self, k1, k2):
        """
        k1 - linear capture point constant
        k2 - angular capture point constant
        """
        self.k1 = k1
        self.k2 = k2

    def set_start_position(self, base_pos, base_orient):
        """
        base_pos - robot body linear position
        base_orient - robot body orientation
        """
        self.base_pos = base_pos
        self.base_orient = base_orient
    
    def get_hip_location(self):
        return self.hip_location

    def step(self, body_pos, R_body, body_lin_vel, body_ang_vel,
                   body_lin_vel_cmd, body_yaw_vel_cmd, body_height_cmd,
                   Tst):
        self.h = body_height_cmd
        twisting_speed_cmd = np.array([0.0, 0.0, body_yaw_vel_cmd], dtype=np.float64)        
        
        # p_hip = (np.array(body_pos)-np.array([0.0, 0.0, self.h])) + R_body @ self.p0_b# -0.015 |0.15 - диаметр шарика
        p_hip = (np.array(body_pos)) + R_body @ self.p0_b# -0.015 |0.15 - диаметр шарика
        p_cross_omega = np.cross(self.p0_b, body_ang_vel)
        p_cross_omega[2] = 0.0
        dp_hip = body_lin_vel + p_cross_omega
        dp_hip_cmd = body_lin_vel_cmd + np.cross(self.p0_b, twisting_speed_cmd)

        self.hip_location = p_hip[:]
        k_raibert = 0.5#1.0#
        raibert_heuristic = k_raibert * Tst * dp_hip_cmd
        capture_point = self.k1 * (dp_hip - dp_hip_cmd)
        centrifugal_term = self.k2 * np.cross(dp_hip, twisting_speed_cmd)

        p_ef_cmd = self.hip_location + raibert_heuristic + capture_point + centrifugal_term - np.array([0.0, 0.0, self.h+0.02])

        return p_ef_cmd
