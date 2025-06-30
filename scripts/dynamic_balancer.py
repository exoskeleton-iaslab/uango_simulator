#!/usr/bin/env python3

import logging
import time

import numpy as np
import rospy
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench, Point, Vector3, PointStamped
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String, Float64

links = {
    "waist": {
        "mass": 10.0,
        "com_offset": np.array([-0.05 - 0.05, 0, 0])
    },
    "LU": {
        "mass": 8.0,
        "com_offset": np.array([0, 0.025, -0.25])
    },
    "RU": {
        "mass": 8.0,
        "com_offset": np.array([0, -0.025, -0.25])
    },
    "LD": {
        "mass": 3.0,
        "com_offset": np.array([0, -0.025, -0.25])
    },
    "RD": {
        "mass": 3.0,
        "com_offset": np.array([0, 0.025, -0.25])
    },
    "LF": {
        "mass": 1.0,
        "com_offset": np.array([0.11 - 0.05, -0.075, -0.0125])
    },
    "RF": {
        "mass": 1.0,
        "com_offset": np.array([0.11 - 0.05, 0.075, -0.0125])
    }
}


class DynamicBalancer:
    def __init__(self):
        rospy.init_node('dynamic_waist_balancer')
        self.filtered_vel = np.zeros(3)
        self.alpha = 0.8

        self.left_upper_leg = 'exoskeleton::LU_link'
        self.right_upper_leg = 'exoskeleton::RU_link'
        self.left_lower_leg = 'exoskeleton::LD_link'
        self.right_lower_leg = 'exoskeleton::RD_link'
        self.left_foot = 'exoskeleton::LF_link'
        self.right_foot = 'exoskeleton::RF_link'
        self.waist_link = 'exoskeleton::waist_link'

        self.kp_pos = np.array([360, 360, 330])  # P gains for position
        self.kd_pos = np.array([350, 350, 320])  # D gains for position
        self.kp_ori = np.array([80, 80, 60])  # P gains for orientation
        self.kd_ori = np.array([60, 60, 60])
        self.gravity = 9.81

        self.pub_com = rospy.Publisher('/exoskeleton/com', PointStamped, queue_size=10)
        self.pub_target = rospy.Publisher('/exoskeleton/target_position', PointStamped, queue_size=10)

        self.latest_state = None
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.callback)
        self.support_leg = ''
        rospy.Subscriber('/exoskeleton/support_leg', String, self.callback_support_leg)

        self.wrench_service = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        rospy.wait_for_service('/gazebo/apply_body_wrench')

    def callback_support_leg(self, msg):
        rospy.loginfo("Received support leg command: %s", msg.data)
        if msg.data == "left":
            self.support_leg = 'left'
        elif msg.data == "right":
            self.support_leg = 'right'
        else:
            self.support_leg = ''

    def callback(self, msg):
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

    def compute_target_position(self):
        position_lf = np.array([
            self.latest_state['LF_pose'].position.x,
            self.latest_state['LF_pose'].position.y,
            self.latest_state['LF_pose'].position.z
        ])
        quat_lf = np.array([
            self.latest_state['LF_pose'].orientation.x,
            self.latest_state['LF_pose'].orientation.y,
            self.latest_state['LF_pose'].orientation.z,
            self.latest_state['LF_pose'].orientation.w
        ])
        position_rf = np.array([
            self.latest_state['RF_pose'].position.x,
            self.latest_state['RF_pose'].position.y,
            self.latest_state['RF_pose'].position.z
        ])
        quat_rf = np.array([
            self.latest_state['RF_pose'].orientation.x,
            self.latest_state['RF_pose'].orientation.y,
            self.latest_state['RF_pose'].orientation.z,
            self.latest_state['RF_pose'].orientation.w
        ])

        r_lf = R.from_quat(quat_lf)
        r_rf = R.from_quat(quat_rf)
        lf = position_lf + r_lf.apply(links['LF']['com_offset'])
        rf = position_rf + r_rf.apply(links['RF']['com_offset'])

        if self.support_leg == 'left':
            target = np.array([lf[0], lf[1], lf[2]])
        elif self.support_leg == 'right':
            target = np.array([rf[0], rf[1], rf[2]])
        else:
            mid_x = (lf[0] + rf[0]) / 2.0
            mid_y = (lf[1] + rf[1]) / 2.0
            max_z = max(lf[2], rf[2])
            target = np.array([mid_x, mid_y, max_z])
        return target

    def com(self):
        total_mass = 0.0
        weighted_com_sum = np.zeros(3)

        for link_name, props in links.items():
            pose = self.latest_state[f"{link_name}_pose"]

            position = np.array([
                pose.position.x,
                pose.position.y,
                pose.position.z
            ])

            quat = np.array([
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ])

            rotation = R.from_quat(quat)
            com_world = position + rotation.apply(props["com_offset"])

            mass = props["mass"]
            total_mass += mass
            weighted_com_sum += mass * com_world

        com_total = weighted_com_sum / total_mass
        weighted_com_vel_sum = np.zeros(3)
        for link_name, props in links.items():
            twist = self.latest_state[f"{link_name}_twist"]
            linear_vel = np.array([
                twist.linear.x,
                twist.linear.y,
                twist.linear.z
            ])

            mass = props["mass"]
            weighted_com_vel_sum += mass * linear_vel

        com_velocity = weighted_com_vel_sum / total_mass
        return com_total, com_velocity, total_mass

    def run(self):
        rate_hz = 100.0
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            if self.latest_state is None:
                rate.sleep()
                continue

            pos, vel, total_mass = self.com()

            target_pos = self.compute_target_position()
            error_pos = target_pos - pos
            self.filtered_vel = self.alpha * self.filtered_vel + (1 - self.alpha) * vel
            d_error_pos = -self.filtered_vel

            force_cmd = self.kp_pos * error_pos + self.kd_pos * d_error_pos
            force_cmd[2] += total_mass * self.gravity

            wrench = Wrench()
            waist_pose = self.latest_state['waist_pose']
            quat = np.array([
                waist_pose.orientation.x,
                waist_pose.orientation.y,
                waist_pose.orientation.z,
                waist_pose.orientation.w
            ])
            r = R.from_quat(quat)
            euler = r.as_euler('xyz')
            waist_twist = self.latest_state['waist_twist']
            angular_vel = np.array([
                waist_twist.angular.x,
                waist_twist.angular.y,
                waist_twist.angular.z
            ])

            torque_cmd = -self.kp_ori * euler - self.kd_ori * angular_vel

            torque_limits = np.array([2000, 1, 2000])
            for i in range(3):
                if abs(torque_cmd[i]) > torque_limits[i]:
                    torque_cmd[i] = np.sign(torque_cmd[i]) * torque_limits[i]

            wrench.torque = Vector3(*torque_cmd)
            wrench.force = Vector3(*force_cmd)

            try:
                self.wrench_service(
                    body_name=self.waist_link,
                    reference_frame="world",
                    reference_point=Point(0, 0, 0),
                    wrench=wrench,
                    start_time=rospy.Time.now(),
                    duration=rospy.Duration(0.05)
                )
            except rospy.ServiceException as e:
                rospy.logwarn("Wrench service failed: %s", e)

            rate.sleep()


if __name__ == '__main__':
    time.sleep(2.0)
    logging.basicConfig(level=logging.INFO)
    pub_left_hip = rospy.Publisher('/exoskeleton/LU_position_controller/command', Float64, queue_size=10)
    pub_right_hip = rospy.Publisher('/exoskeleton/RU_position_controller/command', Float64, queue_size=10)
    pub_left_knee = rospy.Publisher('/exoskeleton/LD_position_controller/command', Float64, queue_size=10)
    pub_right_knee = rospy.Publisher('/exoskeleton/RD_position_controller/command', Float64, queue_size=10)
    pub_left_ankle = rospy.Publisher('/exoskeleton/LF_position_controller/command', Float64, queue_size=10)
    pub_right_ankle = rospy.Publisher('/exoskeleton/RF_position_controller/command', Float64, queue_size=10)

    while not rospy.is_shutdown():
        try:
            controller = DynamicBalancer()
            controller.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS Interrupt Exception caught, restarting controller...")
        except Exception as e:
            rospy.logerr(f"Unexpected error: {e}")

        pub_left_hip.publish(0.0)
        pub_right_hip.publish(0.0)
        pub_left_knee.publish(0.0)
        pub_right_knee.publish(0.0)
        pub_left_ankle.publish(0.0)
        pub_right_ankle.publish(0.0)
