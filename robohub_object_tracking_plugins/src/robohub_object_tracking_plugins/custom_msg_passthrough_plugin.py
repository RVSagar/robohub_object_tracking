from robohub_object_tracking.pose_detection_plugin import BasePoseDetectionPlugin

class CustomMsgPassthroughPlugin(BasePoseDetectionPlugin):

    def __init__(self):
        self._callback = None

    def get_name(self):
        return "Custom"

    def set_callback(self, callback):
        self._callback = callback

    def detect_poses(self, msgs):
        self._callback(self.get_name(), msgs)