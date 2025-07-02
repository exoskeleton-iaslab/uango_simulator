#!/usr/bin/env python3

import logging
import os
import sys
import time

import rospy
from std_msgs.msg import Float64

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from scripts.links import Links
from scripts.dynamic_balancer import DynamicBalancer


class RunBalance:
    def __init__(self):
        time.sleep(2.0)
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

    def zero_all_joints(self):
        self.pub_left_hip.publish(0.0)
        self.pub_right_hip.publish(0.0)
        self.pub_left_knee.publish(0.0)
        self.pub_right_knee.publish(0.0)
        self.pub_left_ankle.publish(0.0)
        self.pub_right_ankle.publish(0.0)

    def run(self):
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
