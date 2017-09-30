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

        self.path_pub = rospy.Publisher('/navigation/waypoints', Path, queue_size=1, latch=True)

        # self.publish_path()

        # rospy.logwarn("pose: %s", path)

        rospy.spin()

    def publish_path(self):
        path = Path()
        path.header.frame_id = "world"
        path.poses.append(
            self.generate_pose(1131.22, 1183.27, 0.1069651, 0.0, 0.0, 0.0436201197059, 0.999048189607))
        path.poses.append(
            self.generate_pose(1151.22, 1203.27, 0.1069651, 0.0, 0.0, 0.0436201197059, 0.999048189607))
        path.poses.append(
            self.generate_pose(1171.22, 1223.27, 0.1069651, 0.0, 0.0, 0.0436201197059, 0.999048189607))
        path.poses.append(
            self.generate_pose(1191.22, 1243.27, 0.1069651, 0.0, 0.0, 0.0436201197059, 0.999048189607))

        self.path_pub.publish(path)


    # def loop(self):
    #     rate = rospy.Rate(1) # 50Hz
    #     while not rospy.is_shutdown():
    #         path = Path()
    #         path.header.frame_id = "world"
    #         path.poses.append(
    #             self.generate_pose(1131.22, 1183.27, 0.1069651, 0.0, 0.0, 0.0436201197059, 0.999048189607))
    #         path.poses.append(
    #             self.generate_pose(1151.22, 1203.27, 0.1069651, 0.0, 0.0, 0.0436201197059, 0.999048189607))
    #         path.poses.append(
    #             self.generate_pose(1171.22, 1223.27, 0.1069651, 0.0, 0.0, 0.0436201197059, 0.999048189607))
    #         path.poses.append(
    #             self.generate_pose(1191.22, 1243.27, 0.1069651, 0.0, 0.0, 0.0436201197059, 0.999048189607))
    #
    #
    #         self.path_pub.publish(path)
    #         rate.sleep()


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

    # On receiving base_waypoints publish a list of paths for Rviz
    def base_waypoints_callback(self, lane):
        path = Path()
        path.header.frame_id = "world"

        for waypoint in lane.waypoints:
            wp_pose = waypoint.pose.pose
            pose = self.generate_pose(wp_pose.position.x, wp_pose.position.y, 0, 0, 0, 0, 0)
            path.poses.append(pose)

        self.path_pub.publish(path)
        pass


if __name__ == '__main__':
    try:
        WolfpackVisualizationHelper()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start wolfpack visualization helper node.')
