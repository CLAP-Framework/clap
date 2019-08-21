import rospy, tf
import math
import json
from collections import namedtuple, defaultdict, OrderedDict

from zzz_cognition_msgs.msg import MapState

from scipy.optimize import linear_sum_assignment

class DecisionBenchmark:
    """
    """
    def __init__(self, include_tags=None, exclude_tags=None):
        # TODO: implement
        self._reported = False
        self._accumulate_acc_error = 0
        self._accumulate_steering_error = 0


    def add_frame(self, gtcmd, percmd, gtdmap, perdmap):

        acc_error, steering_error = self.cmd_effect_metric(gtcmd,percmd)
        self._accumulate_acc_error += abs(acc_error)
        self._accumulate_steering_error += abs(steering_error)

        if gtdmap.model == MapState.MODEL_JUNCTION_MAP:
            pass
        else:
            front_d_error, rear_d_error, front_vehicle_false_nagetive, \
            front_vehicle_false_positive, rear_vehicle_false_nagetive, \
            rear_vehicle_false_positive = self.surrounding_vehicle_state_error(gtdmap, perdmap)


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

    def surrounding_vehicle_state_error(self, gtdmap, perdmap):

        # multilan map

        front_d_error = []
        rear_d_error = []

        front_vehicle_false_nagetive = [] # miss detection
        front_vehicle_false_positive = [] # detect fake vehicle
        rear_vehicle_false_nagetive = []
        rear_vehicle_false_positive = []


        for lane_index, lane_gt in enumerate(gtdmap.mmap.lanes):
            lane_per = perdmap.mmap.lanes[lane_index]
            fvs_gt = lane_gt.front_vehicles
            fvs_per = lane_per.front_vehicles
            rvs_gt = lane_gt.rear_vehicles
            rvs_per = lane_per.rear_vehicles

            if len(fvs_gt) > 0 and len(fvs_per) > 0:
                front_d_error.append(fvs_gt[0].mmap_x - fvs_per[0].mmap_x)
            else:
                front_d_error.append(0)

            if len(fvs_gt) > 0 and len(fvs_per) == 0:
                front_vehicle_false_nagetive.append(True)
            else:
                front_vehicle_false_nagetive.append(False)

            if len(fvs_gt) == 0 and len(fvs_per) > 0:
                front_vehicle_false_positive.append(True)
            else:
                front_vehicle_false_positive.append(False)



            if len(rvs_gt) > 0 and len(rvs_per) > 0:
                rear_d_error.append(rvs_gt[0].mmap_x - rvs_per[0].mmap_x)
            else:
                rear_d_error.append(0)

            if len(rvs_gt) > 0 and len(rvs_per) == 0:
                rear_vehicle_false_nagetive.append(True)
            else:
                rear_vehicle_false_nagetive.append(False)

            if len(rvs_gt) == 0 and len(rvs_per) > 0:
                rear_vehicle_false_positive.append(True)
            else:
                rear_vehicle_false_positive.append(False)

        return front_d_error, rear_d_error, front_vehicle_false_nagetive, \
                front_vehicle_false_positive, rear_vehicle_false_nagetive, \
                rear_vehicle_false_positive



    def cmd_effect_metric(self,gtcmd,percmd):

        acc_error = gtcmd.accel - percmd.accel
        steering_error = gtcmd.steer - percmd.steer

        return acc_error, steering_error

    def vehicles_in_lane_metric(self,vehicles_gt,vehicles_per,key_vehicle_num = 1):
        pass
