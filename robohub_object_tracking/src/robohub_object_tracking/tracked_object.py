from geometry_msgs.msg import Pose

from tracking_systems_list import TrackingSystemsList

class TrackedObject:

    def __init__(self, desired_frame):
        self._frame = desired_frame
        self._pose = Pose()
        self._pose.orientation.w = 1

        self._tracking_points = {TrackingSystemsList.ARUCO: {}, TrackingSystemsList.VICON: {}}

    def add_tracking_point(self, system, id, offset):
        self._tracking_points[system] = {str(id): offset}

    def is_tracked_by(self, system):
        return len(self._tracking_points[system].keys()) > 0

    def has_tracking_point(self, system, id):
        if system not in self._tracking_points.keys():
            return False
        if str(id) not in self._tracking_points[system].keys():
            return False
        
        return True

    def get_tracking_point_offset(self, system, id):
        return self._tracking_points[system][str(id)]

    def update_pose(self, new_pose):
        self._pose = new_pose

    def get_pose(self):
        return self._pose

    def get_frame(self):
        return self._frame
