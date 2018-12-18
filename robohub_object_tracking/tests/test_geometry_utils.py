#!/usr/bin/env python

PKG = 'robohub_object_tracking'
import roslib; roslib.load_manifest(PKG)

import math
import sys
import unittest
import tf

from geometry_msgs.msg import Pose, Point, Quaternion

from  robohub_object_tracking import geometry_utils
    

class TestTransformPose(unittest.TestCase):

    def setUp(self):
        pass

    def create_pose(self):
        p = Pose()
        p.position.x = 0
        p.position.y = 0
        p.position.z = 0
        p.orientation.x = 0
        p.orientation.y = 0
        p.orientation.z = 0
        p.orientation.w = 1
        return p

    def assert_pose_almost_equal(self, pose, vec, quat):
        # Vec (x,y,z), Quat(x,y,z,q) are tuples
        self.assertAlmostEquals(pose.position.x, vec[0])
        self.assertAlmostEquals(pose.position.y, vec[1])
        self.assertAlmostEquals(pose.position.z, vec[2])

        self.assertAlmostEquals(pose.orientation.x, quat[0])
        self.assertAlmostEquals(pose.orientation.y, quat[1])
        self.assertAlmostEquals(pose.orientation.z, quat[2])
        self.assertAlmostEquals(pose.orientation.w, quat[3])

    def test_identity_transformation_base(self):
        base = self.create_pose()
        offset = self.create_pose()

        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (0,0,0), (0,0,0,1))

    def test_identity_transformation_offset(self):
        base = self.create_pose()
        offset = self.create_pose()

        offset.position.x = 1
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (1,0,0), (0,0,0,1))

        offset.position.y = 2
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (1,2,0), (0,0,0,1))

        offset.position.z = 3
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (1,2,3), (0,0,0,1))

    def test_identity_transformation_rotated(self):
        base = self.create_pose()
        offset = self.create_pose()

        offset.orientation.x = 1
        offset.orientation.w = 0
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (0,0,0), (1,0,0,0))

        r = 1.0/math.sqrt(2)
        offset.orientation.x = r
        offset.orientation.y = r
        offset.orientation.w = 0
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (0,0,0), (r,r,0,0))

        r = 1.0/math.sqrt(3)
        offset.orientation.x = r
        offset.orientation.y = r
        offset.orientation.z = r
        offset.orientation.w = 0
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (0,0,0), (r,r,r,0))

        r = 1.0/math.sqrt(4)
        offset.orientation.x = r
        offset.orientation.y = r
        offset.orientation.z = r
        offset.orientation.w = r
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (0,0,0), (r,r,r,r))
    
    def test_translation_of_base(self):
        base = self.create_pose()
        offset = self.create_pose()

        base.position.x = 1
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (1,0,0), (0,0,0,1))

        base.position.y = 2
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (1,2,0), (0,0,0,1))

        base.position.z = 3
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (1,2,3), (0,0,0,1))

    def test_translation_of_both(self):
        base = self.create_pose()
        offset = self.create_pose()

        base.position = Point(*(1, 2, 3))
        offset.position = Point(*(4, 5, 6))
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (5,7,9), (0,0,0,1))

    def test_orientation_of_base_roll(self):
        base = self.create_pose()
        offset = self.create_pose()
        offset.position = Point(*(1, 0, 0))

        quat = tf.transformations.quaternion_from_euler(math.pi*0.25, 0, 0)
        base.orientation = Quaternion(*quat)
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (1,0,0), quat)

        quat = tf.transformations.quaternion_from_euler(-math.pi*0.25, 0, 0)
        base.orientation = Quaternion(*quat)
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (1,0,0), quat)

    def test_orientation_of_base_pitch(self):
        base = self.create_pose()
        offset = self.create_pose()
        offset.position = Point(*(1, 0, 0))

        quat = tf.transformations.quaternion_from_euler(0, math.pi*0.5, 0)
        base.orientation = Quaternion(*quat)
        res = geometry_utils.transform_pose(offset, base)
        print(res)
        print(quat)
        self.assert_pose_almost_equal(res, (0,0,-1), quat)

        quat = tf.transformations.quaternion_from_euler(0, -math.pi*0.5, 0)
        base.orientation = Quaternion(*quat)
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (0,0,1), quat)

    def test_orientation_of_base_yaw(self):
        base = self.create_pose()
        offset = self.create_pose()
        offset.position = Point(*(1, 0, 0))

        quat = tf.transformations.quaternion_from_euler(0, 0, math.pi*0.5)
        base.orientation = Quaternion(*quat)
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (0,1,0), quat)

        quat = tf.transformations.quaternion_from_euler(0, 0, -math.pi*0.5)
        base.orientation = Quaternion(*quat)
        res = geometry_utils.transform_pose(offset, base)
        self.assert_pose_almost_equal(res, (0,-1,0), quat)

    def test_full_transformation(self):
        base = Pose()
        base.position = Point(*(1, 2, 3))
        base.orientation = Quaternion(*tf.transformations.quaternion_from_euler(math.pi*0.5, 0, 0))
    
        offset = Pose()
        offset.position = Point(*(4, 5, 6))
        offset.orientation = Quaternion(*tf.transformations.quaternion_from_euler(math.pi*0.5, 0, 0))

        res = geometry_utils.transform_pose(offset, base)
        quat = tf.transformations.quaternion_from_euler(math.pi, 0, 0)
        self.assert_pose_almost_equal(res, (1+4,-6+2,5+3), quat)



if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'geometry_utils_tests', TestTransformPose)