#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, CustomTrafficLight

import math
import os
import time
import tf
from numpy import random
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

LOOKAHEAD_WPS = 200# Number of waypoints we will publish.
STOP_DISTANCE = 3.00# Distance in 'm' from TL stop line from which the car starts to stop.
STOP_HYST = 3# Margin of error for a stopping car.
SAFE_DECEL_FACTOR = 0.1# Multiplier to the decel limit.
ACC_FACTOR = 0.5# Multiplier to the accel limit

class WaypointUpdater(object):
    def __init__(self):
        # Initialize the node with the Master Process
        rospy.init_node('waypoint_updater')

        # Subscribers
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/traffic_waypoint', CustomTrafficLight, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        # Publishers
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1, latch = True)
       
        # Member variables
        self.car_position = None
        self.car_yaw = None
        self.car_curr_vel = None
        self.cruise_speed = None
        self.car_action = None
        self.closestWaypoint = None
        self.waypoints = []
        self.final_waypoints = []
        self.tl_idx = None
        self.tl_state = None
        self.do_work()

    def do_work(self):
        rate = rospy.Rate(5)
        # ROS parameters
        self.cruise_speed = self.kmph_to_mps(rospy.get_param('~/waypoint_loader/velocity', 40.0))
        self.decel_limit = abs(rospy.get_param('~/twist_controller/decel_limit', -5))
        self.accel_limit = rospy.get_param('~/twist_controller/accel_limit', 1)

        env_velocity = os.getenv('VELOCITY','0')
        if float(env_velocity) > 0.01:
            self.cruise_speed = self.kmph_to_mps(float(env_velocity))

        while not rospy.is_shutdown():
                if (self.car_position != None and self.waypoints != None and self.tl_state != None and self.car_curr_vel != None):
                       self.safe_distance = (self.car_curr_vel ** 2)/(2 * self.decel_limit * SAFE_DECEL_FACTOR)
                       self.closestWaypoint = self.NextWaypoint(self.car_position, self.car_yaw, self.waypoints)
                       if self.tl_idx != None:
                          self.distance_to_tl = self.distance(self.waypoints, self.closestWaypoint, self.tl_idx)
                       self.car_action = self.desired_action(self.tl_idx, self.tl_state, self.closestWaypoint, self.waypoints)
                       self.generate_final_waypoints(self.closestWaypoint, self.waypoints, self.car_action, self.tl_idx)
                       self.publish()
                else:
                       rand = random.uniform(0,1)
                       if self.car_position == None and  rand < 0.01:
                               rospy.logwarn("[WP_UPDATER] /current_pose not received")
                       if self.waypoints == None and rand < 0.01:
                               rospy.logwarn("[WP_UPDATER] /base_waypoints not received")
                       if self.tl_idx == None and rand < 0.01:
                               rospy.logwarn("[WP_UPDATER] /traffic_waypoint not received")
                       if self.car_curr_vel == None  and rand < 0.01:
                               rospy.logwarn("[WP_UPDATER] /current_velocity not received")
                rate.sleep()

    def pose_cb(self, msg):
        car_pose = msg.pose
        self.car_position = car_pose.position 
        car_orientation = car_pose.orientation
        quaternion = (car_orientation.x, car_orientation.y, car_orientation.z, car_orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.car_yaw = euler[2]

    def waypoints_cb(self, msg):
        for waypoint in msg.waypoints:
                self.waypoints.append(waypoint)
        self.base_waypoints_sub.unregister()
        rospy.loginfo("Unregistered from /base_waypoints topic")

    def traffic_cb(self, msg):
        if msg.state == 0:
           self.tl_state = "RED"
           self.tl_idx = msg.waypoint
        elif msg.state == 1:
           self.tl_state = "YELLOW"
           self.tl_idx = msg.waypoint
        elif msg.state == 2:
           self.tl_state = "GREEN"
           self.tl_idx = msg.waypoint
        elif msg.state == 4:
           self.tl_state = "NO"
           self.tl_idx = msg.waypoint

    def current_velocity_cb(self, msg):
        curr_lin = [msg.twist.linear.x, msg.twist.linear.y]
        self.car_curr_vel = math.sqrt(curr_lin[0]**2 + curr_lin[1]**2)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def check_stop(self, tl_index, tl_state, closestWaypoint, dist):
        stop0 = tl_state == "RED" or tl_state == "YELLOW"
        stop1 = (dist < STOP_DISTANCE and stop0) 
        stop2 = (tl_index == closestWaypoint and dist < STOP_DISTANCE and stop0)
        stop3 = (tl_index + STOP_HYST > closestWaypoint and stop0 and dist == 99999)
        return  stop1 or stop2 or stop3

    def check_slow(self, tl_state, dist):
        slow1 = (dist > STOP_DISTANCE and dist < self.safe_distance and tl_state != "NO" and dist != 99999)
        slow2 = (dist > STOP_DISTANCE and dist < 2 * STOP_DISTANCE and tl_state != "NO" and dist != 99999 and self.car_curr_vel > self.mph_to_mps(3.0))
        return  slow1 or slow2

    def check_go(self, tl_index, tl_state, closestWaypoint, dist):
        go1 = (tl_index < closestWaypoint)
        go2 = (tl_state == "GREEN" and dist < STOP_DISTANCE)
        go3 = (dist > self.safe_distance)
        return  go1 or go2 or go3

    def desired_action(self, tl_index, tl_state, closestWaypoint, waypoints): 
        if tl_index != None and tl_state != "NO":
           dist = self.distance_to_tl
           if(self.check_stop(tl_index, tl_state, closestWaypoint, dist)):
              action = "STOP"
              return action
           elif(self.check_slow(tl_state, dist)):
              action = "SLOW"
              return action
           elif(self.check_go(tl_index, tl_state, closestWaypoint, dist)):
              action = "GO"
              return action
        elif tl_index == None or tl_state == "NO" or tl_index == -1:
            if tl_index != -1:
               dist = self.distance_to_tl
               if dist < self.safe_distance:
                  action = "SLOW"
               else:
                  action = "GO"
            else:
               action = "GO"
            return action 

    def stop_waypoints(self, closestWaypoint, waypoints):
        init_vel = self.car_curr_vel
        end = closestWaypoint + LOOKAHEAD_WPS
        if end > len(waypoints) - 1:
           end = len(waypoints) - 1
        for idx in range(closestWaypoint, end):
            velocity = 0.0
            self.set_waypoint_velocity(waypoints, idx, velocity)
            self.final_waypoints.append(waypoints[idx])

    def go_waypoints(self, closestWaypoint, waypoints):
        init_vel = self.car_curr_vel
        end = closestWaypoint + LOOKAHEAD_WPS
        if end > len(waypoints) - 1:
           end = len(waypoints) - 1
        a = ACC_FACTOR * self.accel_limit
        for idx in range(closestWaypoint, end):
            dist = self.distance(waypoints, closestWaypoint, idx+1)
            velocity = math.sqrt(init_vel**2 + 2 * a * dist)
            if velocity > self.cruise_speed:
               velocity = self.cruise_speed
            self.set_waypoint_velocity(waypoints, idx, velocity)
            self.final_waypoints.append(waypoints[idx])

    def slow_waypoints(self, closestWaypoint, tl_index, waypoints):
        dist_to_TL = self.distance_to_tl
        slow_decel = (self.car_curr_vel ** 2)/(2 * dist_to_TL)
        if slow_decel > self.decel_limit:
           slow_decel = self.decel_limit
        init_vel = self.car_curr_vel
        end = closestWaypoint + LOOKAHEAD_WPS
        for idx in range(closestWaypoint, end):
            dist = self.distance(waypoints, closestWaypoint, idx+1)
            if (idx < tl_index):
                vel2 = init_vel ** 2 - 2 * slow_decel * dist
                if vel2 < 0.1:
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
           self.stop_waypoints(closestWaypoint, waypoints)
        elif (action == "SLOW"):
           self.slow_waypoints(closestWaypoint, tl_index, waypoints)
        elif (action == "GO"):
           self.go_waypoints(closestWaypoint, waypoints)

    def publish(self):
        final_waypoints_msg = Lane()
        final_waypoints_msg.header.frame_id = '/World'
        final_waypoints_msg.header.stamp = rospy.Time(0)
        final_waypoints_msg.waypoints = list(self.final_waypoints)
        self.final_waypoints_pub.publish(final_waypoints_msg)
        #v0 = self.get_waypoint_velocity(self.final_waypoints[0])  
        #v1 = self.get_waypoint_velocity(self.final_waypoints[1]) 
        #rospy.logwarn("wp0:%f wp1:%f st:%s d:%f sd:%f",v0,v1,self.car_action,self.distance_to_tl,self.safe_distance)

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
