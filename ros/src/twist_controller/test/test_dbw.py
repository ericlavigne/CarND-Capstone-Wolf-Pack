#!/usr/bin/python

PKG = "twist_controller"
NAME = "test_dbw"

import unittest
import rospy
import time
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport


class TestDBW(unittest.TestCase):
    def __init__(self, *args):
        super(TestDBW, self).__init__(*args)

    def setUp(self):
        rospy.init_node(NAME)

        self.twist_pub = rospy.Publisher('/twist_cmd', TwistStamped, queue_size=10)
        self.dbw_enabled_pub = rospy.Publisher('/vehicle/dbw_enabled', Bool, queue_size=1)

        self.generate_dbw_enabled(False)
        rospy.sleep(0.5)  # wait for stuff to initialize

    def generate_twist(self, x, z):
        twiststamped = TwistStamped()
        twiststamped.twist.linear.x = x
        twiststamped.twist.angular.z = z
        self.twist_pub.publish(twiststamped)

    def generate_dbw_enabled(self, is_enabled):
        enabled = Bool()
        enabled.data = is_enabled
        self.dbw_enabled_pub.publish(enabled)


    # just code to verify how to send the dbw_enabled variable
    def test_sending_dbw_enabled(self):
        self.test_dbw_enabled = False

        def callback(is_enabled):
            self.test_dbw_enabled = True
            # rospy.logwarn('DBW_Enabled: %s', is_enabled)
            # self.assertFalse(is_enabled)

        subscriber = rospy.Subscriber('/vehicle/dbw_enabled', Bool, callback)
        rospy.sleep(0.1)

        self.generate_dbw_enabled(False)

        timeout_t = time.time() + 1.0
        while not rospy.is_shutdown() and not self.test_dbw_enabled and time.time() < timeout_t:
            rospy.sleep(0.1)

        subscriber.unregister()

    # TODO: figure out what the steering, throttle and bake test value should be, hardcoded atm
    # when the values don't match it'll fail multiple times as the callback gets called many times
    # due to the node running at a rate of 50Hz
    def test_simple_dbwsteer(self):
        # check steering
        self.test_simple_steering_called = False
        def steer_callback(steering_cmd):
            self.test_simple_steering_called = True
            # rospy.logwarn('SteeringCmd: %s', steering_cmd)
            self.assertEqual(steering_cmd.steering_wheel_angle_cmd, 0.0)
        steer_sub = rospy.Subscriber('/vehicle/steering_cmd', SteeringCmd, steer_callback)

        # check throttle
        self.test_simple_throttle_called = False
        def callback(throttle_cmd):
            self.test_simple_throttle_called = True
            # rospy.logwarn('ThrottleCmd: %s', throttle_cmd)
            self.assertEqual(throttle_cmd.pedal_cmd, 1.0)
        throttle_sub = rospy.Subscriber('/vehicle/throttle_cmd', ThrottleCmd, callback)

        # check brake
        self.test_simple_brake_called = False
        def callback(brake_cmd):
            self.test_simple_brake_called = True
            # rospy.logwarn('BrakeCmd: %s', brake_cmd)
            self.assertEqual(brake_cmd.pedal_cmd, 0.0)
        brake_sub = rospy.Subscriber('/vehicle/brake_cmd', BrakeCmd, callback)

        # start generating messages to the dbw_node
        rospy.sleep(0.1)
        self.generate_dbw_enabled(True)
        self.generate_twist(11.1112, 3.05018429768e-07)
        timeout_t = time.time() + 1.0
        while not rospy.is_shutdown() and not self.test_simple_steering_called \
                and not self.test_simple_brake_called and not self.test_simple_throttle_called \
                and time.time() < timeout_t:
            rospy.sleep(0.1)

        steer_sub.unregister()
        throttle_sub.unregister()
        brake_sub.unregister()
        self.assertTrue(self.test_simple_steering_called)
        self.assertTrue(self.test_simple_throttle_called)
        self.assertTrue(self.test_simple_brake_called)






if __name__=="__main__":
    import rostest
    rostest.rosrun(PKG, "test_dbw", TestDBW)