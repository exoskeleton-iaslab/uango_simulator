#!/usr/bin/env python3

import logging
import os
import sys

import numpy as np
import rospy
from tf.transformations import quaternion_inverse, quaternion_multiply, euler_from_quaternion
from uango_ros_py.msg import parser, generic

sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))
from scripts.links import Links
from scripts.robot_state_subscriber import RobotStateSubscriber


def create_parser_msg(name, radian_value):
    absolute_position = generic()
    absolute_position.name = "absolute_position"
    absolute_position.value = int(radian_value * 1000)
    absolute_position.function = "absolute_position"

    msg = parser(name=name, messages=[absolute_position])
    return msg


def publisher(robot_state: RobotStateSubscriber):
    rospy.init_node('absolute_position_publisher', anonymous=True)

    pubs = {
        'fem_left': rospy.Publisher('/fem_left', parser, queue_size=10),
        'fem_right': rospy.Publisher('/fem_right', parser, queue_size=10),
        'tib_left': rospy.Publisher('/tib_left', parser, queue_size=10),
        'tib_right': rospy.Publisher('/tib_right', parser, queue_size=10)
    }

    convertion = {
        'fem_left': ['waist', 'LU'],
        'fem_right': ['waist', 'RU'],
        'tib_left': ['LU', 'LD'],
        'tib_right': ['RU', 'RD']
    }

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for pub_name, pub in pubs.items():
            ref, frame = convertion[pub_name]
            pose_frame = robot_state.get_pose(frame)
            pose_ref = robot_state.get_pose(ref)

            q_frame = [pose_frame.orientation.x,
                       pose_frame.orientation.y,
                       pose_frame.orientation.z,
                       pose_frame.orientation.w]
            q_ref = [pose_ref.orientation.x,
                     pose_ref.orientation.y,
                     pose_ref.orientation.z,
                     pose_ref.orientation.w]

            q_ref_inv = quaternion_inverse(q_frame)
            q_relative = quaternion_multiply(q_ref_inv, q_ref)
            _, pitch, _ = euler_from_quaternion(q_relative)

            print(f'{frame}: {np.rad2deg(pitch):.4f} deg, {pitch * 1000:.4f} mrad')

            msg = create_parser_msg(pub_name, pitch)
            pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    links = Links().parse_links()
    robot_state = RobotStateSubscriber(links=links)
    logging.basicConfig(level=logging.INFO)
    try:
        publisher(robot_state)
    except rospy.ROSInterruptException:
        logging.error("ROS Interrupt Exception occurred. Shutting down publisher.")
