from geometry_msgs.msg import PoseStamped

import geometry_utils

class TrackedObject:

    def __init__(self, desired_frame):
        self._frame = desired_frame
        self._pose = PoseStamped()
        self._pose.header.frame_id = desired_frame
        self._pose.pose.orientation.w = 1
        

        self._tracking_points = {}

        self._query_points = {}

    def add_tracking_point(self, system, id, offset):
        self._tracking_points[system] = {str(id): offset}

    def is_tracked_by(self, system):
        if system not in self._tracking_points.keys():
            return False
        return len(self._tracking_points[system].keys()) > 0

    def has_tracking_point(self, system, id):
        if system not in self._tracking_points.keys():
            return False
        if str(id) not in self._tracking_points[system].keys():
            return False
        
        return True

    def get_tracking_point_offset(self, system, id):
        return self._tracking_points[system][str(id)]

    def add_query_point(self, name, pose):
        self._query_points[str(name)] = pose

    def get_query_point_pose(self, name):
        pose = self._query_points[str(name)]

        out = PoseStamped()
        out.header = self.get_pose().header
        out.pose = geometry_utils.transform_pose(pose, self.get_pose())
        return out

    def update_pose(self, new_pose):
        self._pose = new_pose

    def get_pose(self):
        return self._pose

    def get_frame(self):
        return self._frame

    
