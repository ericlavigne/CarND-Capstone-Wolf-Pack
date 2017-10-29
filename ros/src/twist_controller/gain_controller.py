import numpy as np
import random
import rospy
from math import atan2, pi, sqrt

class Regression(object):
    def __init__(self,x_intercept=0.0,slope=1.0,min_slope=None,force_intercept=False):
        self.x_intercept_orig = x_intercept
        self.slope_orig = slope
        self.x_intercept = x_intercept
        self.slope = slope
        self.min_slope = min_slope
        self.force_intercept = force_intercept
        self.recorded_x = []
        self.recorded_y = []
        for i in xrange(100):
            r = random.uniform(-1,1)
            self.recorded_x.append(r)
            self.recorded_y.append(x_intercept + slope * r)
        
    def recalculate(self):
        model = np.polyfit(self.recorded_x,self.recorded_y,1)
        if self.min_slope is None:
            self.slope = model[0]
        else:
            self.slope = max([self.min_slope,model[0]])
        if not self.force_intercept:
            self.x_intercept = model[1]
    
    def predict(self,x):
        return self.x_intercept + self.slope * x
        
    def example(self,x,y):
        self.recorded_x.append(x)
        self.recorded_y.append(y)
        if len(self.recorded_x) % 25 == 0:
            self.recalculate()
        if len(self.recorded_x) > 500:
            self.recorded_x = self.recorded_x[-476:]
            self.recorded_y = self.recorded_y[-476:]

class GainController(object):
    def __init__(self, max_throttle, max_brake, max_steer_angle, steer_ratio):
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        self.max_steer_angle = max_steer_angle
        self.steer_ratio = steer_ratio
        self.steer_gain = Regression(x_intercept=0.0,slope=steer_ratio,min_slope=0.1,force_intercept=True)
        self.throttle_gain = Regression(x_intercept=0.0,slope=1.0,min_slope=0.1,force_intercept=True)
        self.brake_gain = Regression(x_intercept=0.0,slope=1.0,min_slope=0.1,force_intercept=True)

    def control(self, goal_acceleration, goal_angular_velocity,
                      linear_speed, linear_acceleration,
                      angular_velocity,
                      dbw_enabled):
        throttle = self.throttle_gain.predict(goal_acceleration)
        brake = self.brake_gain.predict(0.0 - goal_acceleration)
        steer_angle = self.steer_gain.predict(goal_angular_velocity)
        
        #rospy.logwarn("gain_controller | goal_accel: %s   throttle: %s   brake: %s   accel: %s",
        #              goal_acceleration, throttle, brake, linear_acceleration)
        
        rospy.logwarn("acceleration: %s    goal: %s    throttle: %s    sample: %s    int: %s    slope: %s",
                      linear_acceleration, goal_acceleration, throttle, len(self.throttle_gain.recorded_x),
                      self.throttle_gain.x_intercept, self.throttle_gain.slope)
        
        if throttle < 0.0:
            throttle = 0.0
        if brake < 0.0:
            brake = 0.0
        if throttle > brake:
            brake = 0.0
        else:
            throttle = 0.0
        if throttle > self.max_throttle:
            throttle = self.max_throttle
        if brake > self.max_brake:
            brake = self.max_brake
        if steer_angle > self.max_steer_angle:
            steer_angle = self.max_steer_angle
        if steer_angle < 0 - self.max_steer_angle:
            steer_angle = 0 - self.max_steer_angle
        
        if dbw_enabled and self.last_dbw_enabled:
            if abs(self.last_steer_angle) > 0.001 and linear_speed > 0.1:
                self.steer_gain.example(angular_velocity, self.last_steer_angle)
            if self.last_throttle > 0.0001 and linear_speed > 0.1:
                self.throttle_gain.example(linear_acceleration, self.last_throttle)
            if self.last_brake > 0.0001 and linear_speed > 0.1:
                self.brake_gain.example(0 - linear_acceleration, self.last_brake)
        
        self.last_dbw_enabled = dbw_enabled
        self.last_throttle = throttle
        self.last_brake = brake
        self.last_steer_angle = steer_angle
        
        return throttle, brake, steer_angle
        
