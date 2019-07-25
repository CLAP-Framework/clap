'''
This module provide methods to associate detections to existing trackers

Every method should have as method called associate which takes list of
trackers and detections as input. The result is a list that a list of index
to the trackers.
'''

class ProbabilisticDataAssociationFilter:
    pass

class NearestNeighbor:
    def __init__(self):
        # Parameters setting
        pass

    def associate(self, trackers, detections):
        pass
    pass

NN = NearestNeighbor
PDAF = ProbabilisticDataAssociationFilter
