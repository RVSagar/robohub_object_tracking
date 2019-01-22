

class BasePoseDetectionPlugin:

    def __init__(self):
        pass

    def get_name(self):
        raise NotImplementedError

    def set_callback(self, callback):
        raise NotImplementedError