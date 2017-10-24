#!/usr/bin/env python
import unittest

import sys, os
sys.path.insert(0, sys.path[0]+"/..")  #I hate this

from stability_controller import TwistController

deltat = 0.02

class TestStabilityController(unittest.TestCase):


    def setUp(self):
        self.controller = TwistController(max_angular_velocity=8., accel_limit=1., decel_limit=-5)
        pass

    # control(self, goal_velocity, goal_angular_velocity, current_velocity, deltat, dbw_enabled):

    # Test Acceleration, too simple since the gains are 1, 0, 0
    def test_accel_is_negative_when_current_is_too_high(self):
        acceleration, angular_velocity = self.controller.control(goal_velocity=[10, 0],
                                                                 goal_angular_velocity=4,
                                                                 current_velocity=[12,0], deltat=deltat,
                                                                 dbw_enabled=True)
        self.assertEqual(-2, acceleration)

    def test_accel_is_max_decel_when_current_is_way_too_high(self):
        acceleration, angular_velocity = self.controller.control(goal_velocity=[10, 0],
                                                                 goal_angular_velocity=4,
                                                                 current_velocity=[17,0], deltat=deltat,
                                                                 dbw_enabled=True)
        self.assertEqual(-5, acceleration)

    def test_accel_is_positive_when_current_is_low(self):
        acceleration, angular_velocity = self.controller.control(goal_velocity=[10, 0],
                                                                 goal_angular_velocity=4,
                                                                 current_velocity=[5,0], deltat=deltat,
                                                                 dbw_enabled=True)
        self.assertEqual(1, acceleration)

    def test_accel_is_zero_when_current_is_equal(self):
        acceleration, angular_velocity = self.controller.control(goal_velocity=[10, 0],
                                                                 goal_angular_velocity=4,
                                                                 current_velocity=[10,0], deltat=deltat,
                                                                 dbw_enabled=True)
        self.assertEqual(0, acceleration)



    # test Angular velocity, too simple since the gains are 1, 0, 0
    def test_angular(self):
        acceleration, angular_velocity = self.controller.control(goal_velocity=[10, 0],
                                                                 goal_angular_velocity=4,
                                                                 current_velocity=[10, 0], deltat=deltat,
                                                                 dbw_enabled=True)
        self.assertEqual(4, angular_velocity)
        acceleration, angular_velocity = self.controller.control(goal_velocity=[10, 0],
                                                                 goal_angular_velocity=-5,
                                                                 current_velocity=[10, 0], deltat=deltat,
                                                                 dbw_enabled=True)
        self.assertEqual(-5, angular_velocity)
        acceleration, angular_velocity = self.controller.control(goal_velocity=[10, 0],
                                                                 goal_angular_velocity=-10,
                                                                 current_velocity=[10, 0], deltat=deltat,
                                                                 dbw_enabled=True)
        self.assertEqual(-8, angular_velocity)
        acceleration, angular_velocity = self.controller.control(goal_velocity=[10, 0],
                                                                 goal_angular_velocity=10,
                                                                 current_velocity=[10, 0], deltat=deltat,
                                                                 dbw_enabled=True)
        self.assertEqual(8, angular_velocity)