#!/usr/bin/python
PKG = 'wolfpack_visualisation'
NAME = 'test_visualisation_helper'

import sys
import time
import unittest
import rospy, rostest
from geometry_msgs.msg import PoseStamped
import std_msgs.msg
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray, TrafficLight
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray

class TestVisualisationHelper(unittest.TestCase):
    def __init__(self, *args):
        super(TestVisualisationHelper, self).__init__(*args)

    def setUp(self):
        rospy.init_node(NAME)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoint_loaded_callback)
        rospy.Subscriber('/navigation/waypoints', Path, self.nav_waypoint_loaded_callback)

        self.traffic_lights_pub = rospy.Publisher("/vehicle/traffic_lights", TrafficLightArray, queue_size=1)
        rospy.sleep(0.2)


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


    def test_traffic_light(self):
        self.test_traffic_light_called = False

        def callback(markerarray):
            self.test_traffic_light_called = True
            # rospy.logwarn(markerarray)
            self.assertEqual(len(markerarray.markers), 2)

        rospy.Subscriber("navigation/traffic_light_gt", MarkerArray, callback)
        rospy.sleep(0.5)

        self.publish_traffic_light()

        timeout_t = time.time() + 1.0
        while not rospy.is_shutdown() and not self.test_traffic_light_called and time.time() < timeout_t:
            rospy.sleep(0.1)
        self.assert_(self.test_traffic_light_called)

    def publish_traffic_light(self):

        light_array = TrafficLightArray()
        light_array.header.frame_id = "/world"

        light = TrafficLight()
        light.header.frame_id = "/world"
        light.pose.pose.position.x = 1172.183
        light.pose.pose.position.y = 1186.299
        light.pose.pose.position.z = 5.576891
        light.pose.pose.orientation.z = 0.00061619942315
        light.pose.pose.orientation.w = 0.999999810149
        light.state = TrafficLight.RED
        light_array.lights.append(light)

        light = TrafficLight()
        light.header.frame_id = "/world"
        light.pose.pose.position.x = 1272.183
        light.pose.pose.position.y = 1286.299
        light.pose.pose.position.z = 5.576891
        light.pose.pose.orientation.z = 0.00061619942315
        light.pose.pose.orientation.w = 0.999999810149
        light.state = TrafficLight.GREEN
        light_array.lights.append(light)

        self.traffic_lights_pub.publish(light_array)



if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_visualisation', TestVisualisationHelper, sys.argv)