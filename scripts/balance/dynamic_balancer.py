#!/usr/bin/env python3

import os
import sys

import rospy

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from scripts.balance.robot_state_subscriber import RobotStateSubscriber
from scripts.balance.kinematics import Kinematics
from scripts.balance.controller import Controller
from scripts.balance.wrench_publisher import WrenchPublisher


class DynamicBalancer:
    def __init__(self, links: dict):
        self.state = RobotStateSubscriber(links)
        self.kinematics = Kinematics(links, self.state)
        self.controller = Controller(self.kinematics)
        self.wrench_publisher = WrenchPublisher()

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.state.latest_state is None:
                rate.sleep()
                continue

            force_cmd = self.controller.compute_force_command()
            torque_cmd = self.controller.compute_torque_command()
            self.wrench_publisher.send(force_cmd, torque_cmd)
            rate.sleep()
