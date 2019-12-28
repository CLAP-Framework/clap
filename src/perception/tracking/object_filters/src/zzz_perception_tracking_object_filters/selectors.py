from addict import Dict as edict
from zzz_perception_msgs.msg import DetectionBox, DetectionBoxArray

class CriteriaFilter:
    def __init__(self, **params):
        self._params = edict(params)

    def filter(self, array):
        assert type(array) == DetectionBoxArray

        filtered = DetectionBoxArray()
        filtered.header = array.header

        for target in array.detections:
            if self._size_filter(target):
                filtered.detections.append(target)
        return filtered

    def _size_filter(self, target):
        assert type(target) == DetectionBox

        if target.bbox.dimension.length_x > self._params.max_length_x \
            or target.bbox.dimension.length_x < self._params.min_length_x:
            return False
        if target.bbox.dimension.length_y > self._params.max_length_y \
            or target.bbox.dimension.length_y < self._params.min_length_y:
            return False
        if target.bbox.dimension.length_z > self._params.max_length_z \
            or target.bbox.dimension.length_z < self._params.min_length_z:
            return False
        return True
