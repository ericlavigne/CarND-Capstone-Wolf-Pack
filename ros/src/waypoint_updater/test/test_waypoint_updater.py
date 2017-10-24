#!/usr/bin/python
PKG = 'waypoint_updater'
NAME = 'test_waypoint_updater'

import sys
import time
import unittest
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane

class TestWayPointUpdater(unittest.TestCase):
    def __init__(self, *args):
        super(TestWayPointUpdater, self).__init__(*args)

    def setUp(self):
        rospy.init_node(NAME)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoint_loaded_callback)
        self.pose_pub = rospy.Publisher("/current_pose", PoseStamped, queue_size=10)
        self.traffic_light_pub = rospy.Publisher("/traffic_waypoint", Int32, queue_size=1)
        rospy.sleep(0.5) #wait for stuff to initialize

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

    # Exploring the data of base_waypoints
    def waypoint_loaded_callback(self, waypoints):
        def logwaypoints(waypoints):
            rospy.loginfo("waypoint_loaded_callback: seq: %s time: %s, frame_id: %s", waypoints.header.seq,
                          waypoints.header.stamp, waypoints.header.frame_id)
            rospy.loginfo('waypoints length: %d', len(waypoints.waypoints))
            waypoint = waypoints.waypoints[0]
            rospy.loginfo('pose: %s', waypoint.pose.pose)
            rospy.loginfo('twist: %s', waypoint.twist.twist)
            waypoint = waypoints.waypoints[2]
            rospy.loginfo('pose: %s', waypoint.pose.pose)
            rospy.loginfo('twist: %s', waypoint.twist.twist)
        # logwaypoints(waypoints)

    # Explore the data of pose
    def pose_callback(self, pose_stamped):
        # rospy.loginfo('pose_callback: %s', pose_stamped)
        pass

    # Test that it can find the closest waypoints to the car
    # this test will probably change as we develop the code further
    def test_final_path_on_initial_pose(self):
        self.test_final_path_on_initial_pose_called = False

        def callback(lane):
            self.test_final_path_on_initial_pose_called = True
            # rospy.logwarn(lane)
            waypoint = lane.waypoints[0]
            rospy.logwarn(lane.waypoints[0])
            rospy.logwarn(lane.waypoints[1])
            rospy.logwarn("length: %s", len(lane.waypoints))
            # self.assertEqual(waypoint.pose.pose.position.x, 1131.19)
            # self.assertEqual(waypoint.pose.pose.position.y, 1183.42)
            self.assert_(len(lane.waypoints) > 0)
            # self.assert_(waypoint.pose.pose.position.x==0)

        rospy.Subscriber('/final_waypoints', Lane, callback)
        rospy.sleep(0.5)

        self.traffic_light_pub.publish(5000)
        self.generate_pose(1131.22, 1183.27, 0.1069651, 0.0, 0.0, 0.0436201197059, 0.999048189607)
        timeout_t = time.time() + 2.0
        while not rospy.is_shutdown() and not self.test_final_path_on_initial_pose_called and time.time() < timeout_t:
            rospy.sleep(0.1)

        self.assert_(self.test_final_path_on_initial_pose_called)






if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_waypoint_updater', TestWayPointUpdater)
