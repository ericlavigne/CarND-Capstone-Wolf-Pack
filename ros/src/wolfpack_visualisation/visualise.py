#!/usr/bin/python

import rospy
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray, TrafficLight
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import MarkerArray, Marker

class WolfpackVisualizationHelper(object):
    def __init__(self):
        rospy.init_node("wolfpack_visualization_helper")

        # rospy.Subscriber("/current_pose", PoseStamped, self.current_pose_callback)
        rospy.Subscriber("/base_waypoints", Lane, self.base_waypoints_callback)
        rospy.Subscriber("/final_waypoints", Lane, self.final_waypoints_callback)
        rospy.Subscriber("/vehicle/traffic_lights", TrafficLightArray, self.traffic_light_gt_callback)

        self.path_pub = rospy.Publisher('/navigation/waypoints', Path, queue_size=1, latch=True)
        self.final_path_pub = rospy.Publisher('/navigation/final_waypoints', Path, queue_size=1)
        self.traffic_light_gt_pub = rospy.Publisher("navigation/traffic_light_gt", MarkerArray, queue_size=1)

        self.traffic_light_array_gt = None
        self.publish()


    def publish(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.traffic_light_array_gt is not None:
                marker_array = MarkerArray()

                for index, light in enumerate(self.traffic_light_array_gt.lights):
                    marker = self.generate_light_marker(light, index)
                    marker_array.markers.append(marker)

                self.traffic_light_gt_pub.publish(marker_array)

            rate.sleep()

    def generate_light_marker(self, light, index):
        marker = Marker()
        marker.header.frame_id = "/world"
        marker.ns = "light_gt"
        marker.id = index
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position = light.pose.pose.position
        marker.pose.orientation = light.pose.pose.orientation
        marker.scale.x = 10.0
        marker.scale.y = 10.0
        marker.scale.z = 10.0

        # light = TrafficLight()
        if light.state == TrafficLight.RED:
            marker.color.r = 1.0
            marker.color.g = 0.0
        elif light.state == TrafficLight.GREEN:
            marker.color.r = 0.0
            marker.color.g = 1.0
        elif light.state == TrafficLight.YELLOW:
            marker.color.r = 1.0
            marker.color.g = 1.0
        elif light.state == TrafficLight.UNKNOWN:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0

        marker.color.a = 1.0
        return marker


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

    # def current_pose_callback(self, pose_stamped):
    #     pass

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

    def traffic_light_gt_callback(self, traffic_light_array):
        # rospy.logwarn(traffic_light_array)
        self.traffic_light_array_gt = traffic_light_array



if __name__ == '__main__':
    try:
        WolfpackVisualizationHelper()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start wolfpack visualization helper node.')
