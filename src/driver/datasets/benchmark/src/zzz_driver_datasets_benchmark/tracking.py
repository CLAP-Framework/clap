import rospy, tf
import math
import json
from collections import namedtuple, defaultdict, OrderedDict

from scipy.optimize import linear_sum_assignment
from zzz_common.geometry import polygon_iou, box_to_corners_2d

# TODO: store these information in separate dictionaries
# Type to storage additional fields
class GTBox:
    def __init__(self, trackbox, track_id, tracker, id_switch, fragmentation):
        self.trackbox = trackbox
        self.track_id = track_id
        self.tracker = tracker
        self.id_switch = id_switch
        self.fragmentation = fragmentation

class TRBox:
    def __init__(self, trackbox, track_id, valid):
        self.trackbox = trackbox
        self.track_id = track_id
        self.valid = valid

INF_COST = 1e9
ID_UNTRACKED = -1

class TrackingBenchmark:
    """
    Benchmark results target on KITTI/Detection and KITTI/Raw dataset.

    TODO: Add classification criteria
    TODO: Implement metrics in Smith, Kevin, et al. "Evaluating multi-object tracking." 2005 IEEE Computer Society Conference on Computer Vision and Pattern Recognition (CVPR'05)-Workshops. IEEE, 2005.

    """
    def __init__(self, max_cost=0.5, min_height=25, mostly_lost_threshold=0.2, include_tags=None, exclude_tags=None):
        self._max_cost = max_cost
        self._ml_threshold = mostly_lost_threshold

        self._last_matched_gtid = []
        self._last_matched_trid = []

        self._num_frames = 0
        self._num_truth_boxes = 0
        self._num_tracked_boxes = 0

        self._seq_trajectories = defaultdict(list)
        self._total_cost = 0

        # statistics
        self._tp = 0 # Total true positives
        self._fp = 0 # Total false positives
        self._fn = 0 # Total false negatives
        self._modps = []

        self._reported = False

    def _bounding_box_to_bev_corner(self, box):
        '''
        Convert 3D bounding box into bird's eye view corners
        Box should be in type zzz_perception_msgs/BoundingBox
        '''
        ori = box.pose.pose.orientation
        _,_,yaw = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        return box_to_corners_2d(
            (box.pose.pose.position.x, box.pose.pose.position.y),
            (box.dimension.length_x, box.dimension.length_y),
            yaw
        )


    def cost(self, box1, box2, ctype="overlap"):
        '''
        Box should be in type zzz_perception_msgs/BoundingBox

        TODO: Add more methods for cost criteria: Euclidean distance / Euclidean distance of closest point
        ctype Category:
            overlap: Overlap IoU of the two boxes
            eudist: The euclidean distance between center of two boxes
            boxdist: The euclidean distance between the closest points of two boxes
        '''
        if ctype == "overlap":
            box1 = self._bounding_box_to_bev_corner(box1.bbox)
            box2 = self._bounding_box_to_bev_corner(box2.bbox)
            return polygon_iou(box1, box2)
        elif ctype == "euclidean":
            dx = box1.bbox.pose.pose.position.x - box2.bbox.pose.pose.position.x
            dy = box1.bbox.pose.pose.position.y - box2.bbox.pose.pose.position.y
            dz = box1.bbox.pose.pose.position.z - box2.bbox.pose.pose.position.z
            d = math.sqrt(dx*dx + dy*dy + dz*dz)
            return d # TODO: parametrize threshold
        else:
            raise NotImplementedError("Other cost types haven't been implemented yet!")

    def add_frame(self, result, gt, cost_type="overlap", assign="hungarian", result_filter=None, gt_filter=None):
        '''
        Add result from one frame. Result and GT field are expected to be type of zzz_perception_msgs.msg.TrackingBoxArray

        # TODO: Implement option to select hungarian problem solver or using simple gating?
        '''

        if result.header.stamp - gt.header.stamp > rospy.Duration.from_sec(0.1):
            raise ValueError("Frames added to benchmark should be synchronized in 0.1s!")

        # Convert and filter tracking boxes
        gt_boxes = [GTBox(box, box.uid, 0, 0, 0) for box in gt.targets if (not gt_filter or gt_filter(box))]
        tr_boxes = [TRBox(box, box.uid, False) for box in result.targets if (not gt_filter or gt_filter(box))]
        self._num_frames += 1
        self._num_truth_boxes += len(gt_boxes)
        self._num_tracked_boxes += len(tr_boxes)

        # Use Hungarian method to associate, using boxoverlap 0..1 as cost
        # build cost matrix (Munkres accept only list matrix)
        cost_matrix = []
        cur_matched_gtid = []
        cur_matched_trid = []
        for gtbox in gt_boxes:
            # save current ids
            cur_matched_gtid.append(gtbox.track_id)
            cur_matched_trid.append(ID_UNTRACKED)
            gtbox.tracker       = ID_UNTRACKED
            gtbox.id_switch     = 0
            gtbox.fragmentation = 0
            cost_row = []
            for trbox in tr_boxes:
                cost = self.cost(gtbox.trackbox, trbox.trackbox, ctype=cost_type)
                # gating for boxoverlap
                if cost <= self._max_cost:
                    cost_row.append(cost) # overlap == 1 is cost ==0
                else:
                    cost_row.append(INF_COST) # = 1e9
            cost_matrix.append(cost_row)
            # all ground truth trajectories are initially not associated
            # extend groundtruth trajectories lists (merge lists)
            self._seq_trajectories[gtbox.track_id].append(ID_UNTRACKED)

        if len(gt.targets) is 0:
            cost_matrix=[[]]
        if assign == "hungarian":
            asso_result = linear_sum_assignment(cost_matrix)
        elif assign == "gating":
            raise NotImplementedError()
        else:
            raise ValueError("Unrecoginzed assignment method!")
        association_matrix = list(zip(asso_result[0].tolist(), asso_result[1].tolist()))

        # tmp variables for sanity checks and MODP computation
        cur_tp = 0
        cur_fp = 0
        cur_fn = 0
        cur_c  = 0 # this will sum up the costs for all true positives

        # mapping for tracker ids and ground truth ids
        for gtid, trid in association_matrix:
            # apply gating on cost
            c = cost_matrix[gtid][trid]
            if c < INF_COST:
                gt_boxes[gtid].tracker = tr_boxes[trid].track_id
                cur_matched_trid[gtid] = tr_boxes[trid].track_id
                tr_boxes[trid].valid     = True
                self._total_cost += 1-c
                cur_c += 1-c
                self._seq_trajectories[gt_boxes[gtid].track_id][-1] = tr_boxes[trid].track_id

                # true positives are only valid associations
                self._tp += 1
                cur_tp += 1
            else:
                gt_boxes[gtid].tracker = ID_UNTRACKED
                self._fn += 1
                cur_fn += 1

        cur_fn += len(gt_boxes) - len(association_matrix)
        cur_fp += len(tr_boxes) - cur_tp
        # TODO: assign cur_fp to self._fp

        # sanity checks
        assert cur_tp >= 0, "cur_tp = %f < 0" % cur_tp
        assert cur_fn >= 0, "cur_fn = %f < 0" % cur_fn
        assert cur_fp >= 0, "cur_fp = %f < 0" % cur_fp
        assert cur_tp + cur_fn == len(gt_boxes)
        assert cur_tp + cur_fp == len(tr_boxes)

        # check for id switches or fragmentations
        # XXX: What's the difference to check here or check in report
        for gtpos, gtid in enumerate(cur_matched_gtid):
            if gtid in self._last_matched_gtid:
                trpos = self._last_matched_gtid.index(gtid)
                tid = cur_matched_trid[gtpos]
                lid = self._last_matched_trid[trpos]
                if tid != lid and lid != ID_UNTRACKED and tid != ID_UNTRACKED:
                    gt_boxes[gtpos].id_switch = 1
                    # ids +=1
                if tid != lid and lid != ID_UNTRACKED:
                    gt_boxes[gtpos].fragmentation = 1
                    # fr +=1

        # save current index
        self._last_matched_gtid = cur_matched_gtid
        self._last_matched_trid = cur_matched_trid

        # compute MOTP
        cur_modp = 1
        if cur_tp != 0:
            cur_modp = cur_c / float(cur_tp)
        self._modps.append(cur_modp)

        # Return a accumulate summary
        return {'TP': self._tp, 'FN': self._fn, 'AVG_COST': self._total_cost}

    def doc(self):
        '''
        Get descriptions of the metrics
        '''
        return OrderedDict(
            N="Total count of frames",
            TP="Total count of true positives",
            FP="Total count of false positives",
            FN="Total count of false negatives",
            MT="Percentage of mostly tracked trajectories",
            PT="Percentage of partially tracked trajectories",
            ML="Percentage of mostly lost trajectories",
            precision="precision score, which is TP / (TP + FP)",
            recall="recall score, which is TP / (TP + FN)",
            F1="F1 score",
            FAR="False Acceptance Rate (average FP)",
            IDSW="Count of switches of tracking id in trajectores",
            FRAG="Count of tracking segments with valid id in trajectories",
            MOTA="Multiple Object Tracking Accuracy",
            MODA="Multiple Object Detection Accuracy",
            MOTP="Multiple Object Tracking Precision (total error in estimated position for matched object-hypothesis pairs over all frames)",
            MODP="Multiple Object Detection Precision",
            MOTAL="Multi-object tracking accuracy with log10(id-switches)",
            # TODO: Implement WER and "mme" criteria
            WER="N/A"
        )

    def report(self):
        '''
        Report evaluation statistics, result is in a dictionary
        '''
        if len(self._seq_trajectories) == 0:
            rospy.logerr("Didn't log any trajectories")
        results = OrderedDict(N=self._num_frames, TP=self._tp, FP=self._fp, FN=self._fn)

        cur_mt = cur_ml = cur_pt = cur_id_switches = cur_fragments = 0
        for gtlist in self._seq_trajectories.values():
            # all frames of this gt trajectory are not assigned to any detections
            if all([idx == ID_UNTRACKED for idx in gtlist]):
                cur_ml += 1
                continue

            # compute tracked frames in trajectory
            last_id = gtlist[0]
            tracked = 1 if gtlist[0] >= 0 else 0
            for frame in range(1, len(gtlist)):
                if last_id != gtlist[frame] and last_id != ID_UNTRACKED and gtlist[frame] != ID_UNTRACKED and gtlist[frame-1] != ID_UNTRACKED:
                    cur_id_switches += 1
                if frame < len(gtlist)-1 and gtlist[frame-1] != gtlist[frame] and last_id != ID_UNTRACKED and gtlist[frame] != ID_UNTRACKED and gtlist[frame+1] != ID_UNTRACKED:
                    cur_fragments += 1
                if gtlist[frame] != ID_UNTRACKED:
                    tracked += 1
                    last_id = gtlist[frame]
            # handle last frame; tracked state is handled in for loop (gtlist[frame]!=ID_UNTRACKED)
            if len(gtlist) > 1 and gtlist[frame-1] != gtlist[frame] and last_id != ID_UNTRACKED  and gtlist[frame] != ID_UNTRACKED:
                cur_fragments += 1

            # compute MT/PT/ML
            tracking_ratio = tracked / float(len(gtlist))
            if tracking_ratio > 1 - self._ml_threshold:
                cur_mt += 1
            elif tracking_ratio < self._ml_threshold:
                cur_ml += 1
            else: # self._ml_threshold <= tracking_ratio <= 1-self._ml_threshold
                cur_pt += 1
        results['ML'] = cur_ml
        results['MT'] = cur_mt
        results['PT'] = cur_pt
        results['IDSW'] = cur_id_switches
        results['FRAG'] = cur_fragments

        # precision/recall etc.
        if (self._fp + self._tp) == 0 or (self._tp + self._fn) == 0:
            results['recall'] = 0.
            results['precision'] = 0.
        else:
            results['recall'] = self._tp / float(self._tp + self._fn)
            results['precision'] = self._tp / float(self._fp + self._tp)
        if (results['recall'] + results['precision'])==0:
            results['F1'] = 0.
        else:
            results['F1'] = 2. * (results['precision']*results['recall']) / (results['precision']+results['recall'])
        if self._num_frames == 0:
            results['FAR'] = float('nan')
        else:
            results['FAR'] = self._fp / float(self._num_frames)

        # compute CLEARMOT
        if self._num_frames == 0:
            results['MOTA'] = -float("inf")
            results['MODA'] = -float("inf")
        else:
            results['MOTA']  = 1 - (self._fn + self._fp + cur_id_switches) / float(self._num_frames)
            results['MODA']  = 1 - (self._fn + self._fp) / float(self._num_frames)

        if self._tp==0:
            results['MOTP']  = float("inf")
        else:
            results['MOTP']  = 1. * self._total_cost / self._tp
        if self._num_frames != 0:
            if cur_id_switches == 0:
                results['MOTAL'] = results['MOTA']
            else:
                results['MOTAL'] = 1 - (self._fn + self._fp + math.log10(cur_id_switches)) / float(self._num_frames)
        else:
            results['MOTAL'] = -float("inf")
        if self._num_frames == 0:
            results['MODP'] = float('nan')
        else:
            results['MODP'] = sum(self._modps) / float(self._num_frames)

        self._reported = True
        rospy.logdebug("Tracking benchmark results have been reported")
        return results

    '''
    def getThresholds(self, scores, num_gt, num_sample_pts):
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
    '''

    def __del__(self):
        '''
        Save results to a temporary json file if report haven't been called.
        '''
        if not self._reported:
            results = self.report()
            rospy.logwarn("Results haven't been saved yet!")
            rospy.logwarn("Current result: %s", json.dumps(results))
