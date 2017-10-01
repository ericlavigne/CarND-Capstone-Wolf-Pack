#!/usr/bin/python
PKG = 'wolfpack_visualisation'
NAME = 'test_visualisation_helper'

import sys
import time
import unittest
import rospy, rostest
from geometry_msgs.msg import PoseStamped
import std_msgs.msg
from styx_msgs.msg import Lane, Waypoint
from nav_msgs.msg import Path

class TestVisualisationHelper(unittest.TestCase):
    def __init__(self, *args):
        super(TestVisualisationHelper, self).__init__(*args)

    def setUp(self):
        rospy.init_node(NAME)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoint_loaded_callback)
        rospy.Subscriber('/navigation/waypoints', Path, self.nav_waypoint_loaded_callback)


    def nav_waypoint_loaded_callback(self, path):
        # rospy.logwarn(path)
        pass

    def waypoint_loaded_callback(self, lane):
        # rospy.logwarn(path)
        pass

    def generate_pose(self, px, py, pz, ox, oy, oz, ow):
        pose = PoseStamped()
        pose.pose.position.x = px
        pose.pose.position.y = py
        pose.pose.position.z = pz
        pose.pose.orientation.x = ox
        pose.pose.orientation.y = oy
        pose.pose.orientation.z = oz
        pose.pose.orientation.w = ow
        self.pose_pub.publish(pose)


    # Explore the data of pose
    def pose_callback(self, pose_stamped):
        # rospy.loginfo('pose_callback: %s', pose_stamped)
        pass


    def test_something(self):
        self.assert_(1==1)

    def waypoint_loaded_callback(self, lane):
        # rospy.loginfo('pose_callback: %s', lane)
        pass





if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_visualisation', TestVisualisationHelper, sys.argv)