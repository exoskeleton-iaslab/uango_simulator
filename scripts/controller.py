#!/usr/bin/env python3

import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from scripts.kinematics import Kinematics


class Controller:
    def __init__(self, kinematics: Kinematics):
        self.kinematics = kinematics
        self.alpha = 0.8
        self.filtered_vel = np.zeros(3)

        self.kp_pos = np.array([360, 360, 330])
        self.kd_pos = np.array([350, 350, 320])
        self.kp_ori = np.array([80, 80, 60])
        self.kd_ori = np.array([60, 60, 60])
        self.gravity = 9.81
        self.torque_limits = np.array([2000, 1, 2000])

    def compute_force_command(self) -> np.ndarray:
        pos, vel, total_mass = self.kinematics.center_of_mass_state()
        target = self.kinematics.target_position()

        error = target - pos
        self.filtered_vel = self.alpha * self.filtered_vel + (1 - self.alpha) * vel
        d_error = -self.filtered_vel

        force = self.kp_pos * error + self.kd_pos * d_error
        force[2] += total_mass * self.gravity
        return force

    def compute_torque_command(self) -> np.ndarray:
        pose = self.kinematics.state.get_pose('waist')
        twist = self.kinematics.state.get_twist('waist')

        quat = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        euler = Rotation.from_quat(quat).as_euler('xyz')
        angular_vel = np.array([twist.angular.x, twist.angular.y, twist.angular.z])

        torque = -self.kp_ori * euler - self.kd_ori * angular_vel
        return np.clip(torque, -self.torque_limits, self.torque_limits)
