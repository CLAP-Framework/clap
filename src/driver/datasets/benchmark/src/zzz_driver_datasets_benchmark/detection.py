

class DetectionBenchmark:
    """
    Benchmark results target on KITTI/Detection and KITTI/Raw dataset.
    """
    def __init__(self, min_overlap=0.5, max_truncation=0, min_height=25,
        max_occlusion=2, include_tags=None, exclude_tags=None):
        self._min_overlap = min_overlap
        self._max_truncation = max_truncation
        self._min_height = 25
        self._max_occlusion = 2

        self._track_results = []
        self._track_truths = []

    def add_frame(self, result, gt):
        '''
        Add result from one frame. Result and GT field are expected to be type of zzz_perception_msgs.msg.DetectionBoxArray
        '''
        pass

    def __del__(self):
        pass
