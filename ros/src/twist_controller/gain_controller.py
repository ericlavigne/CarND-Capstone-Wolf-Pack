import rospy
from math import atan2, pi, sqrt

class GainController(object):

    def __init__(self, max_throttle, max_brake, max_steer_angle, delay_seconds, steer_ratio):
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        self.max_steer = max_steer_angle
        self.delay_seconds = delay_seconds
        self.steer_ratio = steer_ratio
        
        self.previous_throttle = 0.0
        self.previous_steer = 0.0

    def control(self, goal_acceleration, goal_yaw_rate,
                      current_speed, current_acceleration,
                      current_yaw_rate,
                      deltat, dbw_enabled):

        delta_throttle = deltat * (self.max_throttle + self.max_brake) / max(self.delay_seconds, 0.02)
        delta_steer = deltat * (self.max_steer * 2) / max(self.delay_seconds, 0.02)

        throttle = self.previous_throttle
        if current_acceleration < goal_acceleration:
            throttle += delta_throttle
        if current_acceleration > goal_acceleration:
            throttle -= delta_throttle
        if throttle > self.max_throttle:
            throttle = self.max_throttle
        if throttle < 0.0 - self.max_brake:
            throttle = 0.0 - self.max_brake
        self.previous_throttle = throttle
        
        steer = self.previous_steer
        if current_yaw_rate < goal_yaw_rate:
            steer += delta_steer
        if current_yaw_rate > goal_yaw_rate:
            steer -= delta_steer
        if steer > self.max_steer:
            steer = self.max_steer
        if steer < 0.0 - self.max_steer:
            steer = 0.0 - self.max_steer
        self.previous_steer = steer
        
        brake = 0
        if throttle < 0.:
            brake = 0. - throttle
            throttle = 0.

        return throttle, brake, steer
        
