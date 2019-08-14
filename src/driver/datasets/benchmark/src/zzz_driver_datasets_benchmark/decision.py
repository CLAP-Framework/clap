import rospy, tf
import math
import json
from collections import namedtuple, defaultdict, OrderedDict

from scipy.optimize import linear_sum_assignment

class DecisionBenchmark:
    """
    """
    def __init__(self, include_tags=None, exclude_tags=None):
        # TODO: implement
        self._reported = False

    def add_frame(self, result, gt, cost="overlap", assign="hungarian" result_filter=None, gt_filter=None):
        pass

    def report(self):
        '''
        Report evaluation statistics, result is in a dictionary
        '''
        results = {}

        rospy.logdebug("Decision benchmark results have been reported")
        return results

    def __del__(self):
        '''
        Save results to a temporary json file if report haven't been called.
        '''
        if not self._reported:
            results = self.report()
            rospy.logwarn("Results haven't been saved yet!")
            rospy.logwarn("Current result: %s", json.dumps(results))
