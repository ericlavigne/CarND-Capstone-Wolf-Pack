#!/usr/bin/python

import rospy
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from nav_msgs.msg import Path, Odometry

class WolfpackVisualizationHelper(object):
    def __init__(self):
        rospy.init_node("wolfpack_visualization_helper")

        rospy.Subscriber("/current_pose", PoseStamped, self.current_pose_callback)
        rospy.Subscriber("/base_waypoints", Lane, self.base_waypoints_callback)
        rospy.Subscriber("/final_waypoints", Lane, self.final_waypoints_callback)

        self.path_pub = rospy.Publisher('/navigation/waypoints', Path, queue_size=1, latch=True)
        self.final_path_pub = rospy.Publisher('/navigation/final_waypoints', Path, queue_size=1)

        # self.publish_path()
        # rospy.logwarn("pose: %s", path)

        rospy.spin()

    # On receiving base_waypoints publish a list of paths for Rviz
    def final_waypoints_callback(self, lane):
        # rospy.logwarn(lane)
        path = Path()
        path.header.frame_id = "world"

        for waypoint in lane.waypoints:
            wp_pose = waypoint.pose.pose
            pose = self.generate_pose(wp_pose.position.x, wp_pose.position.y, 0, 0, 0, 0, 0)
            path.poses.append(pose)

        self.final_path_pub.publish(path)

    # On receiving base_waypoints publish a list of paths for Rviz
    def base_waypoints_callback(self, lane):
        path = Path()
        path.header.frame_id = "world"

        for waypoint in lane.waypoints:
            wp_pose = waypoint.pose.pose
            pose = self.generate_pose(wp_pose.position.x, wp_pose.position.y, 0, 0, 0, 0, 0)
            path.poses.append(pose)

        self.path_pub.publish(path)

    def current_pose_callback(self, pose_stamped):
        pass

    def generate_pose(self, px, py, pz, ox, oy, oz, ow):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = px
        pose.pose.position.y = py
        pose.pose.position.z = pz
        pose.pose.orientation.x = ox
        pose.pose.orientation.y = oy
        pose.pose.orientation.z = oz
        pose.pose.orientation.w = ow
        return pose




if __name__ == '__main__':
    try:
        WolfpackVisualizationHelper()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start wolfpack visualization helper node.')
