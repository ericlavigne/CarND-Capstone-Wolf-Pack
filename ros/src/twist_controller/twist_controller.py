import rospy
from math import atan2, pi, sqrt

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, *args, **kwargs):
        pass

    def control(self, goal_velocity, goal_angular_velocity, current_velocity, dbw_enabled):
        throttle = 1.
        brake = 0.
        
        steer = 0.
        yaw_diff = 0.
        current_speed = sqrt(current_velocity[0]**2 + current_velocity[1]**2)
        goal_speed = sqrt(goal_velocity[0]**2 + goal_velocity[1]**2)
        if current_speed > 1 and goal_speed > 1:
            goal_yaw = atan2(goal_velocity[1],goal_velocity[0])
            current_yaw = atan2(current_velocity[1],current_velocity[0])
            yaw_diff = goal_yaw - current_yaw
            if yaw_diff > pi:
                yaw_diff = yaw_diff - 2 * pi
            if yaw_diff < - pi:
                yaw_diff = yaw_diff + 2 * pi
            if yaw_diff > 0.1:
                steer = 1.
            if yaw_diff < -0.1:
                steer = -1.
            #rospy.logwarn("steer: %s   yawdiff: %s   goalspeed: %s   speed: %s",
            #              steer, yaw_diff, goal_speed, current_speed)
        return throttle, brake, steer

