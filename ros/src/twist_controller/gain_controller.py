import numpy as np
import rospy
from kalman import Kalman
from math import atan2, pi, sqrt
from random import random

# The GainController class uses a parameterized motion model to determine
# appropriate throttle and brake values to achieve the intended acceleration.
# The GainController class uses a Kalman filter to automatically tune those
# parameters.

# Motion model is based on throttle gain (varies with speed), brake gain
# (constant), and drag (varies with speed). The throttle gain and drag
# are estimated as continuous and piecewise linear with parameters to
# control the slope within each of N speed bins.

# [v, b, t0, t1, ..., tN, d1, ..., dN]
#
# At a speed of 0, the throttle gain is t0 and the drag is 0. Parameters
# t1 and d1 represent the slope of throttle gain and drag with respect
# to speed in the first bin. Each later throttle and drag parameter is
# a factor by which the throttle or drag in that bin varies compared to
# the previous bin.

def motion_model_initial_state(num_bins):
    state_size = num_bins * 2 + 3
    # The "factor" parameters start at 1, meaning that each bin is assumed
    # to have the same slopes as the previous bin until data shows otherwise.
    mean = np.ones(state_size)
    # Starting speed is assumed to be 0.
    mean[0] = 0
    # Brake assumed to be strong so that first experiment with brake will
    # press gently.
    mean[1] = 10 # m/s2 max braking deceleration
    # Throttle assumed to be strong so that first experiment with throttle
    # will press gently.
    mean[2] = 10 # m/s2 max acceleration of car at speed of 0
    # Variation of throttle with speed is small, so use slope of 0 as starting point.
    mean[3] = 0
    # Initial drag small because drag has little effect at low speeds.
    mean[num_bins + 3] = 0.1
    # Standard deviations should be high because these are all guesses.
    # Setting all standard deviations to half of the corresponding mean.
    stdev = np.zeros(state_size)
    for i in range(state_size):
        stdev[i] = mean[i] * 0.5 + 0.1
    # But that leaves the starting speed with no uncertainty. Set this high
    # so that we believe the first measurement.
    stdev[0] = 100
    return mean,stdev

def motion_model_throttle_gain(state, bin_size):
    num_bins = (len(state) - 3) / 2
    v = max(0,state[0]) # Negative speeds are out of scope
    gain = state[2] # throttle gain at v=0
    gain_slope = state[3]
    for i in range(num_bins):
        bin_min = i * bin_size
        bin_max = bin_min + bin_size
        if i == (num_bins - 1):
            bin_max = 200 # Last bin grows to include all high speeds
        if i > 0:
            gain_slope *= state[i + 3]
        if v > bin_min:
            gain += (min(v, bin_max) - bin_min) * gain_slope
    return gain

def motion_model_drag(state, bin_size):
    num_bins = (len(state) - 3) / 2
    v = state[0]
    drag = 0
    drag_slope = state[num_bins + 3]
    if v < 0:
        drag = drag_slope * v
    for i in range(num_bins):
        bin_min = i * bin_size
        bin_max = bin_min + bin_size
        if i == (num_bins - 1):
            bin_max = 200 # Last bin grows to include all high speeds
        if i > 0:
            drag_slope *= state[num_bins + i + 3]
        if v > bin_min:
            drag += (min(v, bin_max) - bin_min) * drag_slope
    return drag

def motion_model_predict_acceleration(state, bin_size, throttle, brake):
    brake_deceleration = brake * state[1]
    if state[0] < 0:
        brake_deceleration *= -1
    if abs(state[0]) < 0.01:
         brake_deceleration = 0
    throttle_acceleration = motion_model_throttle_gain(state, bin_size) * throttle
    drag_deceleration = motion_model_drag(state, bin_size)
    return throttle_acceleration - drag_deceleration - brake_deceleration

def motion_model_choose_actuation(state, bin_size, goal_acceleration):
    missing_acceleration = goal_acceleration + motion_model_drag(state, bin_size)
    if missing_acceleration > 0.1 and goal_acceleration > 0.001:
        return missing_acceleration / max(0.1, motion_model_throttle_gain(state, bin_size))
    if missing_acceleration < -0.1 and goal_acceleration < -0.001:
        return missing_acceleration / max(0.1, state[1])
    return 0.0

class GainController(object):
    def __init__(self, max_throttle, max_brake):
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        self.num_bins = 10
        self.bin_size = 10
        mean, stdev = motion_model_initial_state(self.num_bins)
        self.kalman = Kalman(mean,stdev)
        self.throttle = 0.0
        self.brake = 0.0
    def predict(self,deltat, dbw_enabled):
        def predict_fn(input_state_with_noise, output_state):
            state_size = len(output_state)
            accel = 0.0
            if dbw_enabled:
                accel = motion_model_predict_acceleration(input_state_with_noise,
                                                          self.bin_size,
                                                          self.throttle, self.brake)
            accel += input_state_with_noise[state_size]
            v = input_state_with_noise[0] + accel * deltat
            output_state[1:state_size] = input_state_with_noise[1:state_size]
            output_state[0] = v
        self.kalman.predict([(1.0 if dbw_enabled else 5.0)],
                            predict_fn)

    def measure(self, deltat, speed):
        def measure_fn(input_state, output_expected_measurement):
            v = input_state[0]
            output_expected_measurement[0] = v
        speed_stdev = 1.0 / min(1.0, max(0.02, deltat))
        self.kalman.measure([speed], [speed_stdev], measure_fn)

    def control(self, goal_acceleration, linear_speed, deltat, dbw_enabled):
        #print("dbw_enabled: " + str(dbw_enabled))

        self.predict(deltat, dbw_enabled)
        self.measure(deltat, linear_speed)

        self.throttle = motion_model_choose_actuation(self.kalman.x,
                                                      self.bin_size,
                                                      goal_acceleration)
        self.brake = 0
        if self.throttle < 0.:
            self.brake = 0. - self.throttle
            self.throttle = 0.

        #if random() < deltat * 3: # print 3 times per second
        #  rospy.logwarn("goal: %.2f  throttle: %.2f  brake: %.2f" % (goal_acceleration, self.throttle, self.brake))

        return self.throttle, self.brake

