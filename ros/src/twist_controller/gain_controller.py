import rospy
from math import atan2, pi, sqrt

class GainController(object):
    def __init__(self, max_throttle, max_brake, max_steer_angle, delay_seconds, steer_ratio):
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        self.max_steer_angle = max_steer_angle
        self.delay_seconds = delay_seconds
        self.steer_ratio = steer_ratio
    def control(self, goal_acceleration, goal_angular_velocity,
                      linear_speed, angular_velocity,
                      linear_acceleration, angular_acceleration,
                      deltat, dbw_enabled):
        throttle = goal_acceleration
        steer_angle = goal_angular_velocity * self.steer_ratio
        brake = 0
        if throttle < 0.:
            brake = 0. - throttle
            throttle = 0.
        return throttle, brake, steer_angle
        
