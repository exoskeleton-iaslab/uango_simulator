#!/usr/bin/env python3

import logging
import os
import sys

import rospy
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from trio import sleep

sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))

from scripts.balance.links import Links
from scripts.balance.dynamic_balancer import DynamicBalancer


class RunBalance:
    def __init__(self):
        logging.basicConfig(level=logging.INFO)
        rospy.init_node('dynamic_balancer_node', anonymous=True)

        self.pub_left_hip = rospy.Publisher('/exoskeleton/LU_position_controller/command', Float64, queue_size=10)
        self.pub_right_hip = rospy.Publisher('/exoskeleton/RU_position_controller/command', Float64, queue_size=10)
        self.pub_left_knee = rospy.Publisher('/exoskeleton/LD_position_controller/command', Float64, queue_size=10)
        self.pub_right_knee = rospy.Publisher('/exoskeleton/RD_position_controller/command', Float64, queue_size=10)
        self.pub_left_ankle = rospy.Publisher('/exoskeleton/LF_position_controller/command', Float64, queue_size=10)
        self.pub_right_ankle = rospy.Publisher('/exoskeleton/RF_position_controller/command', Float64, queue_size=10)

        self.links = Links().parse_links()
        self.links.pop('camera', None)

    def zero_all_joints(self) -> None:
        self.pub_left_hip.publish(0.0)
        self.pub_right_hip.publish(0.0)
        self.pub_left_knee.publish(0.0)
        self.pub_right_knee.publish(0.0)
        self.pub_left_ankle.publish(0.0)
        self.pub_right_ankle.publish(0.0)

    @staticmethod
    def reset_simulation() -> None:
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
            reset_sim()
            rospy.loginfo("Gazebo simulation reset.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to reset simulation: {e}")

    def run(self):
        self.zero_all_joints()
        sleep(5)
        self.reset_simulation()
        while not rospy.is_shutdown():
            try:
                controller = DynamicBalancer(links=self.links)
                controller.run()
            except rospy.ROSInterruptException:
                rospy.loginfo("ROS Interrupt Exception caught, restarting controller...")
            except Exception as e:
                rospy.logerr(f"Unexpected error: {e}")
            finally:
                self.zero_all_joints()


if __name__ == "__main__":
    node = RunBalance()
    node.run()
