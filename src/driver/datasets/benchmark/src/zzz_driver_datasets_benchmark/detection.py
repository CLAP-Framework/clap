

class DetectionBenchmark:
    """
    Benchmark results target on KITTI/Detection and KITTI/Raw dataset.
    """
    def __init__(self, min_overlap=0.5, max_truncation=0, min_height=25,
        max_occlusion=2, include_tags=None， exclude_tags=None):
        self._min_overlap = min_overlap
        self._max_truncation = max_truncation
        self._min_height = 25
        self._max_occlusion = 2

        self._track_results = []
        self._track_truths = []

    def add_frame(self, result, gt):
        '''
        Add result from one frame. Result and GT field are expected to be type of zzz_percetion_msgs.msg.DetectionBoxArray
        '''
        pass

    def getThresholds(self, scores, num_gt, num_sample_pts=num_sample_pts):
        # based on score of true positive to discretize the recall
        # may not be 11 due to not fully recall the results, all the results point has zero precision
        scores = np.array(scores)
        scores.sort()
        scores = scores[::-1]
        current_recall = 0
        thresholds = []
        for i, score in enumerate(scores):
            l_recall = (i + 1) / float(num_gt)
            if i < (len(scores) - 1):
                r_recall = (i + 2) / float(num_gt)
            else:
                r_recall = l_recall
            if (((r_recall - current_recall) < (current_recall - l_recall)) and (i < (len(scores) - 1))):
                continue

            thresholds.append(score)
            current_recall += 1 / (num_sample_pts - 1.0)

        return thresholds

    def __del__(self):
        # TODO: save results
