#!/usr/bin/env python

PKG = 'robohub_object_tracking'
import roslib; roslib.load_manifest(PKG)

import unittest
import std_msgs

from robohub_object_tracking import TrackedObject
from robohub_object_tracking import TrackingSystemsList

from geometry_msgs.msg import Pose, Point, Quaternion

class TestBasics(unittest.TestCase):

    def setUp(self):
        pass

    def test_initializes(self):
        to = TrackedObject("map")
        self.assertEquals(to._tracking_points, {"Aruco": {}, "Vicon": {}})

    def test_recognizes_tracking_systems(self):
        to = TrackedObject("map")

        self.assertEquals(to.is_tracked_by(TrackingSystemsList.ARUCO), False)
        self.assertEquals(to.is_tracked_by(TrackingSystemsList.VICON), False)

        to.add_tracking_point(TrackingSystemsList.ARUCO, 0, None)
        self.assertEquals(to.is_tracked_by(TrackingSystemsList.ARUCO), True)
        self.assertEquals(to.is_tracked_by(TrackingSystemsList.VICON), False)

        to = TrackedObject("map")
        to.add_tracking_point(TrackingSystemsList.VICON, 0, None)
        self.assertEquals(to.is_tracked_by(TrackingSystemsList.ARUCO), False)
        self.assertEquals(to.is_tracked_by(TrackingSystemsList.VICON), True)

        to = TrackedObject("map")
        to.add_tracking_point(TrackingSystemsList.ARUCO, 0, None)
        to.add_tracking_point(TrackingSystemsList.VICON, 0, None)
        self.assertEquals(to.is_tracked_by(TrackingSystemsList.ARUCO), True)
        self.assertEquals(to.is_tracked_by(TrackingSystemsList.VICON), True)

    def test_has_tracking_point(self):
        to = TrackedObject("map")
        to.add_tracking_point(TrackingSystemsList.ARUCO, 0, "test")

        # Test Existence
        self.assertEquals(to.has_tracking_point(TrackingSystemsList.ARUCO, 0), True)

        # Test Wrong System, Wrong ID
        self.assertEquals(to.has_tracking_point(TrackingSystemsList.VICON, 1), False)

        # Test Wrong System, Correct ID
        self.assertEquals(to.has_tracking_point(TrackingSystemsList.VICON, 0), False)

        # Test Correct System, Wrong ID
        self.assertEquals(to.has_tracking_point(TrackingSystemsList.ARUCO, 1), False)
    
    def test_adds_tracking_points(self):
        to = TrackedObject("map")

        # Easier to test with simple object instead of Pose
        to.add_tracking_point(TrackingSystemsList.ARUCO, 0, "test")
        self.assertEquals(to.get_tracking_point_offset(TrackingSystemsList.ARUCO, 0), "test")

        # Update; Should get replaced
        to.add_tracking_point(TrackingSystemsList.ARUCO, 0, "test again")
        self.assertEquals(to.get_tracking_point_offset(TrackingSystemsList.ARUCO, 0), "test again")

        # New one; Make sure is different
        to.add_tracking_point(TrackingSystemsList.ARUCO, 1, "test third")
        self.assertEquals(to.get_tracking_point_offset(TrackingSystemsList.ARUCO, 1), "test third")

        # Different system. Should allow id overlap
        to.add_tracking_point(TrackingSystemsList.VICON, 0, "test vicon")
        self.assertEquals(to.get_tracking_point_offset(TrackingSystemsList.VICON, 0), "test vicon")

    def test_update_pose(self):
        to = TrackedObject("map")

        to.update_pose("a")
        self.assertEquals(to._pose, "a")

        to.update_pose("b")
        self.assertEquals(to._pose, "b")

    def test_get_pose(self):
        to = TrackedObject("map")

        to._pose = "a"
        self.assertEquals(to.get_pose(), "a")

        to._pose = "b"
        self.assertEquals(to.get_pose(), "b")

    def test_get_frame(self):
        to = TrackedObject("map")
        self.assertEquals(to.get_frame(), "map")

        to = TrackedObject("world")
        self.assertEquals(to.get_frame(), "world")


class TestQueryPoints(unittest.TestCase):

    def setUp(self):
        pass

    def test_add_query_point(self):
        to = TrackedObject("map")

        self.assertEquals(len(to._query_points.keys()), 0)

        to.add_query_point("pt1", None)
        self.assertEquals(len(to._query_points.keys()), 1)

        to.add_query_point("pt1", 3)
        self.assertEquals(len(to._query_points.keys()), 1)

        to.add_query_point("pt2", 4)
        self.assertEquals(len(to._query_points.keys()), 2)
    
    def test_get_query_point(self):
        to = TrackedObject("map")

        p1 = Pose()
        p1.position = Point(*(1, 2, 3))
        p1.orientation = Quaternion(*(0, 0, 0, 1))

        to.add_query_point("p1", p1)
        self.assertEquals(to.get_query_point_pose("p1"), p1)

        p2 = Pose()
        p2.position = Point(*(4, 5, 6))
        p2.orientation = Quaternion(*(0, 0, 0, 1))

        to.add_query_point("p2", p2)
        self.assertEquals(to.get_query_point_pose("p2"), p2)

    def test_query_point_updates_with_base_pose(self):
        to = TrackedObject("map")

        p1 = Pose()
        p1.position = Point(*(1, 2, 3))
        p1.orientation = Quaternion(*(0, 0, 0, 1))

        to.add_query_point("p1", p1)
        self.assertEquals(to.get_query_point_pose("p1"), p1)

        to._pose.position.x = 1
        p2 = Pose()
        p2.position = Point(*(2, 2, 3))
        p2.orientation = Quaternion(*(0, 0, 0, 1))

        self.assertEquals(to.get_query_point_pose("p1"), p2)
    


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'tracked_object_tests', TestBasics)
    rostest.rosrun(PKG, 'tracked_object_tests', TestQueryPoints)