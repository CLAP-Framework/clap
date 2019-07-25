

class VanillaTracker:
    def __init__(self, valid_thres):
        self._tracked_objects = dict() # Object trackers
        self._tracked_features = dict() # Feature trackers (shape, class, etc)
        self._tracked_counter = 0 # Counter for tracked objects
        self._count_track = dict() # Count for frames tracked consecutively
        self._count_lost = dict() # Count for frames lost consecutively
        
        self._valid_thres = valid_thres

    def associate(self, input):
        # 1. prevent existing tracker not to explode by judging the covariance matrix
        # 2. add new 
        pass

    def measurent_validate(self, input):
        
        pass
