#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String


class RobotStateSubscriber:
    def __init__(self, links: dict):
        self.latest_state = None
        self.support_leg = ''
        self.links = links

        self.left_upper_leg = 'exoskeleton::LU_link'
        self.right_upper_leg = 'exoskeleton::RU_link'
        self.left_lower_leg = 'exoskeleton::LD_link'
        self.right_lower_leg = 'exoskeleton::RD_link'
        self.left_foot = 'exoskeleton::LF_link'
        self.right_foot = 'exoskeleton::RF_link'
        self.waist_link = 'exoskeleton::waist_link'

        rospy.Subscriber('/gazebo/link_states', LinkStates, self._callback_link_states)
        rospy.Subscriber('/exoskeleton/support_leg', String, self._callback_support_leg)

    def _callback_support_leg(self, msg: String):
        self.support_leg = msg.data

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

    def get_pose(self, link: str) -> Pose:
        return self.latest_state[f'{link}_pose']

    def get_twist(self, link: str) -> Twist.linear:
        return self.latest_state[f'{link}_twist']

    def get_support_leg(self) -> str:
        return self.support_leg
