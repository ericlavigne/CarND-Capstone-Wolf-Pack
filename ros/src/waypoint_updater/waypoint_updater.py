#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import time
import tf
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

LOOKAHEAD_WPS = 200# Number of waypoints we will publish. You can change this number
STOP_DISTANCE = 2.0# Distance in 'm' from TL stop line where car should halt
SAFE_DECEL_FACTOR = 0.2
UNSAFE_VEL_FACTOR = 0.85
EPSILON = 0.1

class WaypointUpdater(object):
    def __init__(self):
        # Initialize the node with the Master Process
        rospy.init_node('waypoint_updater')

        # Subscribers
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        # Publishers
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1, latch = True)
       
        # Member variables
        self.car_pose = None
        self.car_position = None
        self.car_orientation = None
        self.car_yaw = None
        self.car_curr_vel = None
        self.slow_decel = None
        self.cruise_speed = None
        self.unsafe_speed = None
        self.unsafe_distance = None
        self.car_action = None
        self.prev_action = None
        self.closestWaypoint = None
        self.waypoints = []
        self.final_waypoints = []
        self.tl_idx = None
        self.tl_state = None
        self.init_slow = False #flag set in slow_waypoints() and reset in desired_action()
        self.do_work()

    def do_work(self):
        rate = rospy.Rate(30)
        # ROS parameters
        self.cruise_speed = self.kmph_to_mps(rospy.get_param('~/waypoint_loader/velocity', 64.0))
        self.decel_limit = abs(rospy.get_param('~/twist_controller/decel_limit', -5))
        self.accel_limit = rospy.get_param('~/twist_controller/accel_limit', 1)
        self.unsafe_distance = (self.cruise_speed ** 2)/(2 * self.decel_limit)
        self.unsafe_speed = self.cruise_speed * UNSAFE_VEL_FACTOR
        while not rospy.is_shutdown():
                if (self.car_position != None and self.waypoints != None and self.tl_state != None and self.car_curr_vel != None):
                       self.safe_distance = (self.car_curr_vel ** 2)/(2 * self.decel_limit * SAFE_DECEL_FACTOR)
                       self.closestWaypoint = self.NextWaypoint(self.car_position, self.car_yaw, self.waypoints)
                       self.car_action = self.desired_action(self.tl_idx, self.tl_state, self.closestWaypoint, self.waypoints)
                       self.generate_final_waypoints(self.closestWaypoint, self.waypoints, self.car_action, self.tl_idx)
                       self.publish()
                else:
                       if self.car_position == None:
                               rospy.logwarn("/current_pose not received")
                       if self.waypoints == None:
                               rospy.logwarn("/base_waypoints not received")
                       if self.tl_idx == None:
                               rospy.logwarn("/traffic_waypoint not received")
                       if self.car_curr_vel == None:
                               rospy.logwarn("/current_velocity not received")
                rate.sleep()

    def pose_cb(self, msg):
        self.car_pose = msg.pose
        self.car_position = self.car_pose.position 
        self.car_orientation = self.car_pose.orientation
        quaternion = (self.car_orientation.x, self.car_orientation.y, self.car_orientation.z, self.car_orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.car_yaw = euler[2]

    def waypoints_cb(self, msg):
        for waypoint in msg.waypoints:
                self.waypoints.append(waypoint)
        self.base_waypoints_sub.unregister()
        rospy.loginfo("Unregistered from /base_waypoints topic")

    def traffic_cb(self, msg):
        if msg.data != -1:
           self.tl_idx = msg.data
           self.tl_state = "RED"
        else:
           self.tl_state = "GREEN"

    def current_velocity_cb(self, msg):
        curr_lin = [msg.twist.linear.x, msg.twist.linear.y]
        self.car_curr_vel = math.sqrt(curr_lin[0]**2 + curr_lin[1]**2)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def check_stop(self, tl_index, tl_state, closestWaypoint, dist):
        stop1 = (tl_index > closestWaypoint and tl_index < closestWaypoint + LOOKAHEAD_WPS and dist < STOP_DISTANCE and tl_state == "RED") 
        stop2 = (tl_index == closestWaypoint and dist < STOP_DISTANCE and tl_state == "RED")
        return  stop1 or stop2 

    def check_slow(self, tl_state, dist):
        slow1 = (dist < self.safe_distance and dist > self.unsafe_distance)
        slow2 = (dist > STOP_DISTANCE and dist < self.unsafe_distance and tl_state == "RED" and self.safe_distance > EPSILON)
        return  slow1 or slow2 

    def check_go(self, tl_index, tl_state, closestWaypoint, dist):
        go1 = (dist > self.safe_distance and tl_index > closestWaypoint)
        go2 = (dist > self.safe_distance and tl_index > closestWaypoint + LOOKAHEAD_WPS)
        go3 = (tl_state == "GREEN" and dist < self.unsafe_distance) or (tl_index < closestWaypoint)
        go4 = (tl_state == "GREEN" and self.prev_action == "STOP")
        return  go1 or go2 or go3 or go4

    def desired_action(self, tl_index, tl_state, closestWaypoint, waypoints): 
        if tl_index != None:
           dist = self.distance(waypoints, closestWaypoint, tl_index)
           #rospy.loginfo("Distance: %f, Curr Speed: %f", dist,self.car_curr_vel)
           #rospy.logwarn("Unsafe Distance: %f", self.unsafe_distance)
           #rospy.logwarn("Traffic Light Index: %d", tl_index)
           #rospy.loginfo("Curr Speed: %f",self.car_curr_vel)
           if(self.check_stop(tl_index, tl_state, closestWaypoint, dist)):
              action = "STOP"
              self.prev_action = "STOP"
              self.init_slow = False
              return action
           elif(self.check_slow(tl_state, dist)):
              action = "SLOW"
              self.prev_action = "SLOW"
              return action
           elif(self.check_go(tl_index, tl_state, closestWaypoint, dist)):
              action = "GO"
              self.prev_action = "GO"
              self.init_slow = False
              return action
        else:
            action = "GO"
            self.prev_action = "GO"
            self.init_slow = False
            return action 

    def stop_waypoints(self, closestWaypoint, waypoints):
        velocity = 0.0
        end = closestWaypoint + LOOKAHEAD_WPS
        if end > len(waypoints) - 1:
           end = len(waypoints) - 1
        for idx in range(closestWaypoint, closestWaypoint + LOOKAHEAD_WPS):
            self.set_waypoint_velocity(waypoints, idx, velocity)
            self.final_waypoints.append(waypoints[idx])

    def go_waypoints(self, closestWaypoint, waypoints):
        init_vel = self.car_curr_vel
        end = closestWaypoint + LOOKAHEAD_WPS
        if end > len(waypoints) - 1:
           end = len(waypoints) - 1
        for idx in range(closestWaypoint, end):
            dist = self.distance(waypoints, closestWaypoint, idx+1)
            velocity = math.sqrt(init_vel**2 + 2 * self.accel_limit * dist)
            if velocity > self.cruise_speed:
               velocity = self.cruise_speed
            self.set_waypoint_velocity(waypoints, idx, velocity)
            self.final_waypoints.append(waypoints[idx])

    def slow_waypoints(self, closestWaypoint, tl_index, waypoints):
        if(self.init_slow == False):
            dist_to_TL = self.distance(waypoints, closestWaypoint, tl_index)
            #x1 = self.car_position.x
            #y1 = self.car_position.y
            #x2 = waypoints[tl_index].pose.pose.position.x
            #y2 = waypoints[tl_index].pose.pose.position.y
            #dist_to_TL = self.distance_any(x1, y1, x2, y2)
            self.slow_decel = (self.car_curr_vel ** 2)/(2 * dist_to_TL)
            if self.slow_decel > self.decel_limit:
               self.slow_decel = self.decel_limit
            self.init_slow = True
        for idx in range(closestWaypoint, closestWaypoint + LOOKAHEAD_WPS):
            dist = self.distance(waypoints, idx, tl_index)
            if (idx < tl_index):
                velocity = math.sqrt(2*self.slow_decel*dist)
                if (dist < self.unsafe_distance and self.car_curr_vel > self.unsafe_speed):
                   vel2 = self.car_curr_vel ** 2 - 2*self.decel_limit*dist
                   if vel2 < EPSILON:
                      vel2 = 0.0
                   velocity = math.sqrt(vel2)
                self.set_waypoint_velocity(waypoints, idx, velocity)
                self.final_waypoints.append(waypoints[idx])
            else:
                velocity = 0.0
                self.set_waypoint_velocity(waypoints, idx, velocity)
                self.final_waypoints.append(waypoints[idx])

    def generate_final_waypoints(self, closestWaypoint, waypoints, action, tl_index):
        self.final_waypoints = []
        if (action == "STOP"):
           #rospy.logwarn(action)
           self.stop_waypoints(closestWaypoint, waypoints)
        elif (action == "SLOW"):
           #rospy.logwarn(action)
           self.slow_waypoints(closestWaypoint, tl_index, waypoints)
        elif (action == "GO"):
           #rospy.logwarn(action)
           self.go_waypoints(closestWaypoint, waypoints)

    def publish(self):
        final_waypoints_msg = Lane()
        final_waypoints_msg.header.frame_id = '/World'
        final_waypoints_msg.header.stamp = rospy.Time(0)
        final_waypoints_msg.waypoints = list(self.final_waypoints)
        self.final_waypoints_pub.publish(final_waypoints_msg)    

    def closest_waypoint(self, position, waypoints):
        closestLen = float("inf")
        closestWaypoint = 0
        dist = 0.0
        for idx in range(0, len(waypoints)):
                x = position.x
                y = position.y
                map_x = waypoints[idx].pose.pose.position.x
                map_y = waypoints[idx].pose.pose.position.y
                dist = self.distance_any(x, y, map_x, map_y)
                if (dist < closestLen):
                        closestLen = dist
                        closestWaypoint = idx
        return closestWaypoint

    def NextWaypoint(self, position, yaw, waypoints):
        closestWaypoint = self.closest_waypoint(position, waypoints)
        map_x = waypoints[closestWaypoint].pose.pose.position.x
        map_y = waypoints[closestWaypoint].pose.pose.position.y
        heading = math.atan2((map_y - position.y), (map_x - position.x))
        angle = abs(yaw - heading)
        if (angle > math.pi/4):
                  closestWaypoint += 1
                  if (closestWaypoint > len(waypoints)-1):
                             closestWaypoint -= 1
        return closestWaypoint 

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        if wp2 >= wp1:
           dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
           for i in range(wp1, wp2+1):
               dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
               wp1 = i
        else:
           dist = 99999
        return dist

    def distance_any(self, x1, y1, x2, y2):
        return math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))

    def kmph_to_mps(self, kmph):
        return 0.278 * kmph

    def mph_to_mps(self, mph):
        return 0.447 * mph


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
