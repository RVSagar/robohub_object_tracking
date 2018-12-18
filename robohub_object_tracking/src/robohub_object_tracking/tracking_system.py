from aruco_msgs.msg import Marker, MarkerArray

class TrackingSystem:

    def __init__(self):
        self.marker_array_topic = "/robohub/object_tracking/aruco_markers"
        self._tracked_objects = []

    def add_tracked_object(self, tracked_object):
        self._tracked_objects.append(tracked_object)
 
    def vicon_callback(self, msg):
        pass
    
    def aruco_callback(self, msg):
        pass

    def update_objects(self, system, tracked_id, new_pose):
        for obj in self._tracked_objects:
            if obj.has_tracking_point(system, tracked_id):
                self.update_object_pose(obj, system, tracked_id, new_pose)

    def update_object_pose(self, obj, system, id, new_pose):
        tracking_offset = obj.get_tracking_point_offset(system, tracked_id)

        obj_base_pose = self.calculated_base_pose(new_pose, tracking_offset)
        obj_pose = self.transform_pose_to_frame(obj_base_pose, obj.get_frame())

        obj.update_pose(obj_pose)


    def calculate_base_pose(self, tracked_pose, offset):
        pass

    def transform_pose_to_frame(self, pose, frame):
        pass
        
