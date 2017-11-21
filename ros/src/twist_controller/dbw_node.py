#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from stability_controller import TwistController
from gain_controller import GainController


from dynamic_reconfigure.server import Server
from twist_controller.cfg import PIDParamsConfig

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)
        self.dbw_enabled = False

        self.twist_controller = TwistController(max_steer_angle, accel_limit, decel_limit)
        self.gain_controller = GainController(max_throttle=1.0, max_brake=1.0, max_steer_angle=max_steer_angle,
                                              delay_seconds=1.0, steer_ratio=steer_ratio)

        self.goal_acceleration = 0
        self.goal_yaw_rate = 0.
        self.current_linear = [0,0]

        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_callback)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_callback)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_callback)

        srv = Server(PIDParamsConfig, self.config_callback)

        self.loop()

    def config_callback(self, config, level):
        rospy.logwarn("Updating Steering PID %s, %s, %s", config["Steer_P"], config["Steer_I"], config["Steer_D"])
        rospy.logwarn("Updating Throttle PID %s, %s, %s", config["Throttle_P"], config["Throttle_I"], config["Throttle_D"])
        self.twist_controller.update_steer_pid(config["Steer_P"], config["Steer_I"], config["Steer_D"])
        self.twist_controller.update_throttle_pid(config["Throttle_P"], config["Throttle_I"], config["Throttle_D"])
        return config

    def twist_cmd_callback(self, msg):
        new_goal_acceleration = msg.twist.linear.x
        if new_goal_acceleration < 0 and self.goal_acceleration > 0:
            self.twist_controller.reset_throttle_pid()
        #rospy.logwarn("Updating intended acceleration: %s", new_goal_acceleration)

        self.goal_acceleration = new_goal_acceleration
        self.goal_yaw_rate = msg.twist.angular.z

    def current_velocity_callback(self, msg):
        self.current_linear = [msg.twist.linear.x, msg.twist.linear.y]

    def dbw_enabled_callback(self, msg):
        self.dbw_enabled = msg.data

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Calculate these using pose topic and rospy.get_time()
            linear_speed = 0.0
            angular_velocity = 0.0
            linear_acceleration = 0.0
            angular_acceleration = 0.0
            deltat = 0.02

            goal_linear_acceleration, goal_angular_velocity = self.twist_controller.control(self.goal_acceleration,
                                                                                            self.goal_yaw_rate,
                                                                                            self.current_linear,
                                                                                            deltat,
                                                                                            self.dbw_enabled)

            # rospy.logwarn("c:%.2f, g:%.2f, o:%.2f", self.current_linear[0],
            #               self.goal_linear[0], goal_linear_acceleration)

            #if(self.goal_linear[0] != 0 and goal_linear_acceleration < 0 and goal_linear_acceleration > -self.brake_deadband):
            #    goal_linear_acceleration = 0

            throttle, brake, steering = self.gain_controller.control(goal_linear_acceleration, goal_angular_velocity,
                                                                     linear_speed, angular_velocity,
                                                                     linear_acceleration, angular_acceleration,
                                                                     deltat, self.dbw_enabled)

            if brake > 0:
                brake = brake * BrakeCmd.TORQUE_MAX / -self.decel_limit

            # rospy.logwarn("c:%.2f, g:%.2f, o:%.2f, b:%.2f", self.current_linear[0],
            #               self.goal_linear[0], goal_linear_acceleration, brake)

            if self.dbw_enabled:
                self.publish(throttle, brake, steering)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT # range [0,1]
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

if __name__ == '__main__':
    DBWNode()

