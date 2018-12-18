import tf
import numpy as np

from geometry_msgs.msg import Pose, Point, Quaternion

# Euler should be RPY (https://github.com/ros/geometry/blob/hydro-devel/tf/src/tf/transformations.py)

def transform_pose(offset, base):
    # Transforms offset to be a pose with respect to base's frame
    # In matrix terms: result = [base][offset]
    base_mat = create_matrix_from_pose_msg(base)
    offset_mat = create_matrix_from_pose_msg(offset)

    final_mat = np.dot(base_mat, offset_mat)

    return create_pose_msg_from_matrix(final_mat)


def create_pose_msg_from_matrix(matrix):
    q = tf.transformations.quaternion_from_matrix(matrix)
    t = tf.transformations.translation_from_matrix(matrix)

    p = Pose()
    p.position = Point(*t)
    p.orientation = Quaternion(*q)
    return p

def create_matrix_from_pose_msg(pose):
    t = (pose.position.x, pose.position.y, pose.position.z)
    o = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

    trans = tf.transformations.translation_matrix(t)
    rot = tf.transformations.quaternion_matrix(o)

    mat = np.dot(trans, rot)
    return mat