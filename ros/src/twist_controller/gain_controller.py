import numpy as np
import random
import rospy
import time
from math import atan2, pi, sqrt
from scipy.optimize import curve_fit

def steer_model(x, steer_ratio):
    return np.multiply(x[0], (0.01 + abs(steer_ratio)))
    
def throttle_model(min_v):
    # force goes toward accelerating (x[0]) and maintaining velocity (x[1]).
    # Higher velocities require much more throttle to maintain.
    return lambda x, a, b, c: abs(a) + (0.0001 + abs(b)) * x[0] + (0.0001 + abs(c)) * (x[1] - min_v)

def brake_model(x, a):
    return np.multiply(x[0], (0.01 + abs(a)))

class Regression(object):
    def __init__(self, model_fn, model, verbosity=0, name="unnamed"):
        self.model_fn = model_fn
        self.model = model
        self.name = name
        self.num_recordings = 0
        self.max_recorded = 200
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
        start_time = time.time()
        try:
            model, covariances = curve_fit(self.model_fn, self.recorded_xv, self.recorded_y, p0=self.model)
            self.model = model
        except RuntimeError:
            rospy.logwarn("!!!!! Unable to recalculate " + self.name + " !!!!!")
        end_time = time.time()
        time_elapsed = end_time - start_time
        if self.verbosity >= 1:
            rospy.logwarn("recalculated %s model in %s seconds: %s", self.name, time_elapsed, self.model)
    
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
        if self.verbosity >= 2 and random.uniform(0,1) < 0.01:
            rospy.logwarn("predict %s:   x=%s  v=%s  =>  %s", self.name, x, v, res[0])
        return res[0]
        
    def example(self,x,v,y):
        if self.verbosity >= 2 and random.uniform(0,1) < 0.01:
            rospy.logwarn("Example %s:   x=%s   v=%s   y=%s", self.name, x, v, y)
        times_to_record = 1
        if self.num_recordings < self.max_recorded:
            times_to_record = 10
        for _ in range(times_to_record):
            i = random.randint(0,self.max_recorded - 1)
            self.recorded_xv[0,i] = x
            self.recorded_xv[1,i] = v
            self.recorded_y[i] = y
        self.num_recordings += 1
        if self.num_recordings == (self.max_recorded / 10):
            self.recalculate()
        if self.num_recordings == (self.max_recorded / 5):
            self.recalculate()
        if self.num_recordings % self.max_recorded == 0:
            self.recalculate()

class GainController(object):
    def __init__(self, max_throttle, max_brake, max_steer_angle, steer_ratio):
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        self.max_steer_angle = max_steer_angle
        self.steer_ratio = steer_ratio
        #self.steer_gain = Regression(model_fn=steer_model, model=[steer_ratio],verbosity=1)
        self.throttle_gains = [Regression(model_fn=throttle_model(0.0), model=[0.0, 0.1, 0.1], \
                                          name="throttle0", verbosity=0)]
        self.brake_gains = [Regression(model_fn=brake_model, model=[1.0], \
                                       name="brake0",verbosity=0)]

    def control(self, goal_acceleration, goal_steering, #goal_radians_per_meter,
                      linear_speed, linear_acceleration,
                      angular_velocity,
                      dbw_enabled):
        velocity_index = max(0, int(linear_speed) / 10)
        while velocity_index + 1 > len(self.throttle_gains):
            min_speed = velocity_index * 10
            old = self.throttle_gains[-1].model
            new = [old[0] + abs(old[2]) * 10, old[1], old[2]]
            rospy.logwarn("Creating throttle model for speed %s: %s", min_speed, new)
            self.throttle_gains.append(Regression(model_fn=throttle_model(min_speed), model=new, \
                                                  name=("throttle" + str(velocity_index)), verbosity=0))
            old = self.brake_gains[-1].model
            new = [old[0]]
            rospy.logwarn("Creating brake model for speed %s: %s", min_speed, new)
            self.brake_gains.append(Regression(model_fn=brake_model, model=new, \
                                               name=("brake" + str(velocity_index)), verbosity=0))
            
        throttle_gain = self.throttle_gains[velocity_index]
        brake_gain = self.brake_gains[velocity_index]
            
        #radians_per_meter = angular_velocity / max(1.0, linear_speed)
        throttle = 0.0
        brake = 0.0
        if goal_acceleration > 0.0:
            throttle = max(0.0, throttle_gain.predict(goal_acceleration, linear_speed))
        if goal_acceleration < -0.1:
            brake = max(0.0, brake_gain.predict(-1 * goal_acceleration, linear_speed))
        #steer_angle = self.steer_gain.predict(goal_radians_per_meter, linear_speed)
        steer_angle = goal_steering * self.steer_ratio
        
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
                throttle_gain.example(linear_acceleration, linear_speed, self.last_throttle)
            if self.last_brake > 0.0001 and linear_acceleration < -0.1:
                brake_gain.example(-1 * linear_acceleration, linear_speed, self.last_brake)
        
        self.last_dbw_enabled = dbw_enabled
        self.last_throttle = throttle
        self.last_brake = brake
        self.last_steer_angle = steer_angle
        
        #if random.uniform(0,1) < 0.01:
        #    rospy.logwarn("goal: %s    actual: %s    throttle: %s    brake: %s",
        #                  goal_acceleration, linear_acceleration, throttle, brake)
        
        return throttle, brake, steer_angle
        
