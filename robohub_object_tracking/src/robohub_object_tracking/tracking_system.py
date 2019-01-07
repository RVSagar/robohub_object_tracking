import rospy

from tf import TransformListener
from tracking_systems_list import TrackingSystemsList
import geometry_utils
from geometry_msgs.msg import PoseStamped
from robohub_object_tracking.msg import TrackedObjectPoseList
from aruco_msgs.msg import Marker, MarkerArray

class TrackingSystem:

    def __init__(self):
        self.marker_array_topic = "/robohub/object_tracking/aruco_markers"
        self._tracked_objects = []

        self._vicon_subscriber = None
        self._aruco_subscriber = None
        self._custom_subscriber = None

        self._tf_listener = None

    def initialize_subscribers(self):
        self.custom_subscriber = rospy.Subscriber("robohub_object_tracking/custom_tracked_objects", TrackedObjectPoseList, self.custom_tracking_callback)

    def initialize_transform_listener(self):
        self._tf_listener = TransformListener()

    def add_tracked_object(self, tracked_object):
        self._tracked_objects.append(tracked_object)
 
    def vicon_callback(self, msg):
        # Frame of measurements comes from TODO
        pass
    
    def aruco_callback(self, msg):
        # Frame of measuremets comes from TODO
        pass

    def custom_tracking_callback(self, msg):
        objects = [(tracked.id, tracked.pose) for tracked in msg.object_list]
        self.perform_updates_for_tracked_objects_list(TrackingSystemsList.CUSTOM, objects)

    def perform_updates_for_tracked_objects_list(self, system, objects):
        # Expect list of tuples (id, pose)
        for obj in objects:
            self.update_objects(system, obj[0], obj[1])

    def update_objects(self, system, tracked_id, new_pose):
        for obj in self._tracked_objects:
            if obj.has_tracking_point(system, tracked_id):
                self.update_object_pose(obj, system, tracked_id, new_pose)

    def update_object_pose(self, obj, system, tracked_id, new_pose):
        tracking_offset = obj.get_tracking_point_offset(system, tracked_id)
        obj_base_pose = self.calculate_base_pose(new_pose, tracking_offset)
        obj_pose = self.transform_pose_to_frame(obj_base_pose, obj.get_frame())
        obj.update_pose(obj_pose)


    def calculate_base_pose(self, tracked_pose, offset):
        m_new = geometry_utils.transform_pose(geometry_utils.calculate_inverse_pose(offset), tracked_pose)
        new_pose = PoseStamped()
        new_pose.header.frame_id = tracked_pose.header.frame_id
        new_pose.pose = m_new

        return new_pose

    def transform_pose_to_frame(self, pose, frame):
        return self._tf_listener.transformPose(frame, pose)
        
