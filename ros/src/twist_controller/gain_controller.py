import numpy as np
import random
import rospy
from math import atan2, pi, sqrt
from scipy.optimize import curve_fit

def steer_model(x, steer_ratio):
    return np.multiply(x[0], (0.01 + abs(steer_ratio)))
    
def throttle_model(x, a, b):
    # force goes toward accelerating (x[0]) and maintaining velocity (x[1]).
    # Higher velocities require much more throttle to maintain.
    return (0.01 + abs(a)) * x[0] + (0.0001 + abs(b)) * x[1]

class Regression(object):
    def __init__(self, model_fn, model, verbosity=0):
        self.model_fn = model_fn
        self.model = model
        self.num_recordings = 0
        self.max_recorded = 500
        self.verbosity = 0
        
        self.recorded_xv = np.zeros((2,self.max_recorded))
        self.recorded_y = np.zeros(self.max_recorded)
        for i in xrange(self.max_recorded):
            x = random.uniform(-1,1)
            v = random.uniform(-2,2)
            self.recorded_xv[0,i] = x
            self.recorded_xv[1,i] = v
            self.recorded_y[i] = self.predict(x,v)
        
        self.verbosity = verbosity
        self.recalculate()
        
    def recalculate(self):
        try:
            model, covariances = curve_fit(self.model_fn, self.recorded_xv, self.recorded_y, p0=self.model)
            self.model = model
        except RuntimeError:
            rospy.logwarn("!!!!! Unable to recalculate gain model !!!!!")
        if self.verbosity >= 1:
            rospy.logwarn("recalculated model: %s", self.model)
    
    def predict(self,x,v):
        res = None
        model_input = np.array([[x],[v]])
        # This if/elif/else is crude replacement for splat operator which Python 2 doesn't have.
        if len(self.model) == 1:
            res = self.model_fn(model_input, self.model[0])
        elif len(self.model) == 2:
            res = self.model_fn(model_input, self.model[0], self.model[1])
        elif len(self.model) == 3:
            res = self.model_fn(model_input, self.model[0], self.model[1], self.model[2])
        else:
            raise Exception("Unrecognized model size " + str(len(self.model)))
        if self.verbosity >= 2:
            rospy.logwarn("predict   x:%s  v:%s  =>  %s", x, v, res[0])
        return res[0]
        
    def example(self,x,v,y):
        if self.verbosity >= 2:
            rospy.logwarn("Example:   x=%s   v=%s   y=%s", x, v, y)
        times_to_record = 1
        if self.num_recordings < self.max_recorded:
            times_to_record = 20
        for _ in range(times_to_record):
            i = random.randint(0,self.max_recorded - 1)
            self.recorded_xv[0,i] = x
            self.recorded_xv[1,i] = v
            self.recorded_y[i] = y
        self.num_recordings += 1
        if self.num_recordings % 50 == 0:
            self.recalculate()

class GainController(object):
    def __init__(self, max_throttle, max_brake, max_steer_angle, steer_ratio):
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        self.max_steer_angle = max_steer_angle
        self.steer_ratio = steer_ratio
        #self.steer_gain = Regression(model_fn=steer_model, model=[steer_ratio],verbosity=1)
        self.throttle_gain = Regression(model_fn=throttle_model, model=[1.0, 0.1],verbosity=1)
        self.brake_gain = Regression(model_fn=throttle_model, model=[1.0, 0.0],verbosity=0)

    def control(self, goal_acceleration, goal_steering, #goal_radians_per_meter,
                      linear_speed, linear_acceleration,
                      angular_velocity,
                      dbw_enabled):
        #radians_per_meter = angular_velocity / max(1.0, linear_speed)
        throttle = self.throttle_gain.predict(goal_acceleration, linear_speed)
        brake = self.brake_gain.predict(goal_acceleration, linear_speed)
        #steer_angle = self.steer_gain.predict(goal_radians_per_meter, linear_speed)
        steer_angle = goal_steering * self.steer_ratio
        
        if throttle > 0.0:
            brake = 0.0
        if throttle < 0.0:
            throttle = 0.0
        if brake > 0.0:
            brake = 0.0
        if throttle > self.max_throttle:
            throttle = self.max_throttle
        if (brake * -1) > self.max_brake:
            brake = self.max_brake * -1.0
        if steer_angle > self.max_steer_angle:
            steer_angle = self.max_steer_angle
        if steer_angle < 0 - self.max_steer_angle:
            steer_angle = 0 - self.max_steer_angle
        
        if dbw_enabled and self.last_dbw_enabled and linear_speed > 0.1:
            #if abs(self.last_steer_angle) > 0.001:
            #    self.steer_gain.example(radians_per_meter, linear_speed, self.last_steer_angle)
            if self.last_throttle > 0.0001:
                self.throttle_gain.example(linear_acceleration, linear_speed, self.last_throttle)
            if self.last_brake < -0.0001:
                self.brake_gain.example(linear_acceleration, linear_speed, self.last_brake)
        
        self.last_dbw_enabled = dbw_enabled
        self.last_throttle = throttle
        self.last_brake = brake
        self.last_steer_angle = steer_angle
        
        #rospy.logwarn("gain output: throttle=%s brake=%s steer_angle=%s", throttle, (0.0 - brake), steer_angle)
        
        if random.uniform(0,1) < 0.01:
            rospy.logwarn("goal: %s    actual: %s    throttle: %s    brake: %s",
                          goal_acceleration, linear_acceleration, throttle, (0.0 - brake))
        
        return throttle, (0.0 - brake), steer_angle
        
