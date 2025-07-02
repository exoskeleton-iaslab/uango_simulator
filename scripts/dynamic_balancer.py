#!/usr/bin/env python3

import numpy as np
import rospy
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench, Point, Vector3
from scipy.spatial.transform import Rotation
from std_msgs.msg import String


class DynamicBalancer:
    def __init__(self, links: dict):
        self.filtered_vel = np.zeros(3)
        self.alpha = 0.8
        self.torque_limits = np.array([2000, 1, 2000])

        self.left_upper_leg = 'exoskeleton::LU_link'
        self.right_upper_leg = 'exoskeleton::RU_link'
        self.left_lower_leg = 'exoskeleton::LD_link'
        self.right_lower_leg = 'exoskeleton::RD_link'
        self.left_foot = 'exoskeleton::LF_link'
        self.right_foot = 'exoskeleton::RF_link'
        self.waist_link = 'exoskeleton::waist_link'

        self.kp_pos = np.array([360, 360, 330])
        self.kd_pos = np.array([350, 350, 320])
        self.kp_ori = np.array([80, 80, 60])
        self.kd_ori = np.array([60, 60, 60])
        self.gravity = 9.81

        self.links = links

        self.latest_state = None
        rospy.Subscriber('/gazebo/link_states', LinkStates, self._callback_link_states)
        self.support_leg = ''
        rospy.Subscriber('/exoskeleton/support_leg', String, self._callback_support_leg)

        self.wrench_service = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        rospy.wait_for_service('/gazebo/apply_body_wrench')

    def _callback_support_leg(self, msg: String):
        if msg.data == "left":
            self.support_leg = 'left'
        elif msg.data == "right":
            self.support_leg = 'right'
        else:
            self.support_leg = ''

    def _callback_link_states(self, msg: LinkStates):
        try:
            ru_idx = msg.name.index(self.right_upper_leg)
            lu_idx = msg.name.index(self.left_upper_leg)
            rd_idx = msg.name.index(self.right_lower_leg)
            ld_idx = msg.name.index(self.left_lower_leg)
            lf_idx = msg.name.index(self.left_foot)
            rf_idx = msg.name.index(self.right_foot)
            waist_idx = msg.name.index(self.waist_link)
        except ValueError:
            return

        self.latest_state = {
            'RU_pose': msg.pose[ru_idx],
            'LU_pose': msg.pose[lu_idx],
            'RD_pose': msg.pose[rd_idx],
            'LD_pose': msg.pose[ld_idx],
            'LF_pose': msg.pose[lf_idx],
            'RF_pose': msg.pose[rf_idx],
            'waist_pose': msg.pose[waist_idx],
            'waist_twist': msg.twist[waist_idx],
            'LF_twist': msg.twist[lf_idx],
            'RF_twist': msg.twist[rf_idx],
            'RU_twist': msg.twist[ru_idx],
            'LU_twist': msg.twist[lu_idx],
            'RD_twist': msg.twist[rd_idx],
            'LD_twist': msg.twist[ld_idx]
        }

    def _extract_pose(self, prefix: str) -> (np.ndarray, np.ndarray):
        pos = self.latest_state[f'{prefix}_pose'].position
        ori = self.latest_state[f'{prefix}_pose'].orientation
        position = np.array([pos.x, pos.y, pos.z])
        quat = np.array([ori.x, ori.y, ori.z, ori.w])
        return position, quat

    def _compute_com_position(self, prefix: str) -> np.ndarray:
        position, quat = self._extract_pose(prefix)
        rotation = Rotation.from_quat(quat)
        com_offset = self.links[prefix]['com_offset']
        return position + rotation.apply(com_offset)

    def _target_position(self) -> np.ndarray:
        lf_com_pos = self._compute_com_position('LF')
        rf_com_pos = self._compute_com_position('RF')

        if self.support_leg == 'left':
            return lf_com_pos
        elif self.support_leg == 'right':
            return rf_com_pos
        else:
            mid_xy = (lf_com_pos[:2] + rf_com_pos[:2]) / 2.0
            max_z = max(float(lf_com_pos[2]), float(rf_com_pos[2]))
            return np.array([mid_xy[0], mid_xy[1], max_z])

    def _compute_link_com_position(self, link_name: str) -> np.ndarray:
        return self._compute_com_position(link_name)

    def _compute_link_linear_velocity(self, link_name: str) -> np.ndarray:
        twist = self.latest_state[f"{link_name}_twist"]
        return np.array([twist.linear.x, twist.linear.y, twist.linear.z])

    def _compute_center_of_mass_state(self) -> (np.ndarray, np.ndarray, float):
        total_mass = 0.0
        weighted_com_pos_sum = np.zeros(3)
        weighted_com_vel_sum = np.zeros(3)

        for link_name, props in self.links.items():
            com_pos = self._compute_link_com_position(link_name)
            linear_vel = self._compute_link_linear_velocity(link_name)

            total_mass += props["mass"]
            weighted_com_pos_sum += props["mass"] * com_pos
            weighted_com_vel_sum += props["mass"] * linear_vel

        com_position = weighted_com_pos_sum / total_mass
        com_velocity = weighted_com_vel_sum / total_mass

        return com_position, com_velocity, total_mass

    def _compute_force_command(self, pos: np.ndarray, vel: np.ndarray, total_mass: float) -> np.ndarray:
        error_pos = self._target_position() - pos
        self.filtered_vel = self.alpha * self.filtered_vel + (1 - self.alpha) * vel
        d_error_pos = -self.filtered_vel

        force_cmd = self.kp_pos * error_pos + self.kd_pos * d_error_pos
        force_cmd[2] += total_mass * self.gravity
        return force_cmd

    def _compute_torque_command(self) -> np.ndarray:
        waist_pose = self.latest_state['waist_pose']
        quat = np.array([
            waist_pose.orientation.x,
            waist_pose.orientation.y,
            waist_pose.orientation.z,
            waist_pose.orientation.w
        ])
        r = Rotation.from_quat(quat)
        euler = r.as_euler('xyz')

        waist_twist = self.latest_state['waist_twist']
        angular_vel = np.array([
            waist_twist.angular.x,
            waist_twist.angular.y,
            waist_twist.angular.z
        ])

        torque_cmd = -self.kp_ori * euler - self.kd_ori * angular_vel
        torque_cmd = np.clip(torque_cmd, -self.torque_limits, self.torque_limits)

        return torque_cmd

    def _send_wrench_command(self, force_cmd: np.ndarray, torque_cmd: np.ndarray) -> None:
        wrench = Wrench()
        wrench.force = Vector3(*force_cmd)
        wrench.torque = Vector3(*torque_cmd)

        try:
            self.wrench_service(
                body_name=self.waist_link,
                reference_frame="world",
                reference_point=Point(0, 0, 0),
                wrench=wrench,
                start_time=rospy.Time.now(),
                duration=rospy.Duration.from_sec(0.05)
            )
        except rospy.ServiceException as e:
            rospy.logwarn(f"Wrench service failed: {e}")

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.latest_state is None:
                rate.sleep()
                continue

            pos, vel, total_mass = self._compute_center_of_mass_state()
            force_cmd = self._compute_force_command(pos, vel, total_mass)
            torque_cmd = self._compute_torque_command()

            self._send_wrench_command(force_cmd, torque_cmd)

            rate.sleep()
