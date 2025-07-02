#!/usr/bin/env python3

import numpy as np
import rospy
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench, Point, Vector3


class WrenchPublisher:
    def __init__(self):
        self.waist_link = 'exoskeleton::waist_link'
        self.wrench_service = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        rospy.wait_for_service('/gazebo/apply_body_wrench')

    def send(self, force: np.ndarray, torque: np.ndarray) -> None:
        wrench = Wrench()
        wrench.force = Vector3(*force)
        wrench.torque = Vector3(*torque)

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
