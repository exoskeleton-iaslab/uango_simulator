#!/usr/bin/env python3

import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from scripts.robot_state_subscriber import RobotStateSubscriber


class Kinematics:
    def __init__(self, links: dict, state_subscriber: RobotStateSubscriber):
        self.links = links
        self.state = state_subscriber

    def _get_com_position(self, link: str) -> np.ndarray:
        pose = self.state.get_pose(link)
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        orientation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        rotation = Rotation.from_quat(orientation)
        return position + rotation.apply(self.links[link]['com_offset'])

    def _get_linear_velocity(self, link: str) -> np.ndarray:
        twist = self.state.get_twist(link)
        return np.array([twist.linear.x, twist.linear.y, twist.linear.z])

    def center_of_mass_state(self) -> (np.ndarray, np.ndarray, float):
        total_mass = 0.0
        weighted_pos = np.zeros(3)
        weighted_vel = np.zeros(3)

        for link, props in self.links.items():
            com_pos = self._get_com_position(link)
            vel = self._get_linear_velocity(link)
            mass = props['mass']

            total_mass += mass
            weighted_pos += mass * com_pos
            weighted_vel += mass * vel

        return weighted_pos / total_mass, weighted_vel / total_mass, total_mass

    def target_position(self) -> np.ndarray:
        lf = self._get_com_position('LF')
        rf = self._get_com_position('RF')
        support_leg = self.state.get_support_leg()

        if support_leg == 'left':
            return lf
        elif support_leg == 'right':
            return rf
        else:
            mid_xy = (lf[:2] + rf[:2]) / 2.0
            max_z = max(float(lf[2]), float(rf[2]))
            return np.array([mid_xy[0], mid_xy[1], max_z])
