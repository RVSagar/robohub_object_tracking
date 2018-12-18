#!/usr/bin/env python

PKG = 'robohub_object_tracking'
import roslib; roslib.load_manifest(PKG)

import unittest
import std_msgs

from robohub_object_tracking import TrackingSystem
from robohub_object_tracking import TrackingSystemsList

class TestAll(unittest.TestCase):

    def setUp(self):
        pass
    
    def create_tracking_system(self):
        ts = TrackingSystem()
        return ts

    def test_initializes(self):
        ts = self.create_tracking_system()

    def test_add_objects(self):
        ts = self.create_tracking_system()
        self.assertEquals(len(ts._tracked_objects), 0)

        ts.add_tracked_object("mock_object")
        self.assertEquals(len(ts._tracked_objects), 1)

        ts.add_tracked_object("mock_object")
        self.assertEquals(len(ts._tracked_objects), 2)



if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'tracking_system_tests', TestAll)