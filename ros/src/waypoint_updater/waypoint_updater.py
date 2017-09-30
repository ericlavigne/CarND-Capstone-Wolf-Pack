#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import time
'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.shortest_distance_index = -1
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        # rospy.loginfo("pose_cb called")
        # rospy.logwarn("warning: pose")
        # rospy.logdebug("Debug: pose_cb called2")
        # if(self.waypoint_index):
        def distance(x, y, x1, y1):
            return math.sqrt((x-x1)*(x-x1) + (y-y1)*(y-y1))

        pos_x = msg.pose.position.x
        pos_y = msg.pose.position.y

        time_t = time.time()

        if(self.base_waypoints is not None):
            shortest_distance = 9999999999.0
            self.shortest_distance_index = 0
            for i in range(len(self.base_waypoints.waypoints)):
                waypoint = self.base_waypoints.waypoints[i]
                x1 = waypoint.pose.pose.position.x
                y1 = waypoint.pose.pose.position.y
                d = distance(pos_x, pos_y, x1, y1)
                if d < shortest_distance:
                    shortest_distance = d
                    self.shortest_distance_index = i

            # rospy.logwarn('shortest_distance_index: %d', self.shortest_distance_index)

            # TODO: should use LOOKAHEAD_WPS instead of 20
            waypoints = self.base_waypoints.waypoints[self.shortest_distance_index:self.shortest_distance_index+20]
            # rospy.logwarn(waypoints)

            lane = Lane()
            lane.waypoints.extend(waypoints)

            self.final_waypoints_pub.publish(lane)
        # rospy.logwarn('time taken for pose_cb: %f', time.time() - time_t)
        # pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
