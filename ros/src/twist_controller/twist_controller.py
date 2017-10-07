import rospy
from math import atan2, pi, sqrt
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, *args, **kwargs):
        self.steer_pid = PID(1., 0., 0., -1., 1.)
        self.throttle_pid = PID(1., 0., 0., -1., 1.)

    def control(self, goal_velocity, goal_angular_velocity, current_velocity, dbw_enabled):
        deltat = 0.02
        
        current_speed = sqrt(current_velocity[0]**2 + current_velocity[1]**2)
        goal_speed = sqrt(goal_velocity[0]**2 + goal_velocity[1]**2)
        speed_diff = goal_speed - current_speed
        throttle = self.throttle_pid.step(speed_diff, deltat)
        
        steer = self.steer_pid.step(goal_angular_velocity, deltat)
        
        if dbw_enabled:
            self.throttle_pid.reset()
            self.steer_pid.reset()
        
        brake = 0.
        if throttle < 0.:
            brake = 0. - throttle
            throttle = 0.
        
        #rospy.logwarn("twist_controller | speed_diff: %s   throttle: %s   brake: %s   angular: %s   steer: %s",
        #      speed_diff, throttle, brake, goal_angular_velocity, steer)
        
        return throttle, brake, steer

