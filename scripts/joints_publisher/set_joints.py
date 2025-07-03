#!/usr/bin/env python3

import os
import sys
import time

import numpy as np
import rospy

from geometry_msgs.msg import Pose
from tf.transformations import quaternion_inverse, quaternion_multiply, euler_from_quaternion
from uango_ros_py.msg import parser, generic

sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))
from scripts.links import Links
from scripts.robot_state_subscriber import RobotStateSubscriber


class SetJoints:
    def __init__(self):
        rospy.init_node('absolute_position_publisher')
        self.robot_state = self._init_robot_state()
        self.publishers = self._create_publishers()
        self.frame_mapping = self._get_joint_frames_mapping()

    @staticmethod
    def _init_robot_state() -> RobotStateSubscriber:
        links = Links().parse_links()
        return RobotStateSubscriber(links=links)

    @staticmethod
    def _create_publishers() -> dict:
        topics = ['fem_left', 'fem_right', 'tib_left', 'tib_right']
        return {
            topic: rospy.Publisher(f'/{topic}', parser, queue_size=10)
            for topic in topics
        }

    @staticmethod
    def _get_joint_frames_mapping() -> dict:
        return {
            'fem_left': ['waist', 'LU'],
            'fem_right': ['waist', 'RU'],
            'tib_left': ['LU', 'LD'],
            'tib_right': ['RU', 'RD']
        }

    @staticmethod
    def _compute_relative_pitch(pose_ref: Pose, pose_frame: Pose) -> float:
        q_frame = [pose_frame.orientation.x, pose_frame.orientation.y,
                   pose_frame.orientation.z, pose_frame.orientation.w]
        q_ref = [pose_ref.orientation.x, pose_ref.orientation.y,
                 pose_ref.orientation.z, pose_ref.orientation.w]
        q_frame_inv = quaternion_inverse(q_frame)
        q_relative = quaternion_multiply(q_frame_inv, q_ref)
        _, pitch, _ = euler_from_quaternion(q_relative)
        return pitch

    @staticmethod
    def _create_absolute_position_message(joint_name: str, radian_value: float) -> parser:
        msg = generic(
            name="absolute_position",
            value=int(radian_value * 1000),
            function="absolute_position"
        )
        return parser(name=joint_name, messages=[msg])

    def _update_and_publish_all(self):
        for joint_name, publisher in self.publishers.items():
            ref_frame, target_frame = self.frame_mapping[joint_name]

            pose_ref = self.robot_state.get_pose(ref_frame)
            pose_target = self.robot_state.get_pose(target_frame)

            pitch = self._compute_relative_pitch(pose_ref, pose_target)

            rospy.logdebug(f"{target_frame}: {np.rad2deg(pitch):.2f}° / {pitch * 1000:.2f} mrad")

            msg = self._create_absolute_position_message(joint_name, pitch)
            publisher.publish(msg)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                self._update_and_publish_all()
                rate.sleep()
            except rospy.ROSTimeMovedBackwardsException:
                pass
            except rospy.ROSInterruptException as e:
                rospy.logwarn(f"ROS Exception: {e}", exc_info=True)
            except Exception as e:
                rospy.logwarn(f"Unexpected error: {e}", exc_info=True)


if __name__ == '__main__':
    rospy.loginfo("Starting SetJoints controller...")
    time.sleep(10)
    rospy.loginfo("SetJoints controller initialized.")
    controller = SetJoints()
    controller.run()
