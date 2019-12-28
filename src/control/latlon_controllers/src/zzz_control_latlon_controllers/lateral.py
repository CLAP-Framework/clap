import rospy
import math
import numpy as np
import tf

from zzz_common.geometry import dense_polyline2d

class PurePersuitLateralController():
    """
    PurePersuitLateralController implements lateral control using a PurePersuit Control.

    TODO(zhcao): Parametrize this controller
    """

    def __init__(self):
        pass

    def run_step(self, ego_pose, trajectory, current_speed):
        """
        Execute one step of lateral control to steer the vehicle towards a certain waypoin.
        """

        control_point = self._control_point(ego_pose, trajectory, current_speed)
        if len(control_point) < 2:
            return 0.0
        return self._purepersuit_control(control_point, ego_pose)

    def _control_point(self, ego_pose, trajectory, current_speed, resolution = 0.1):

        if current_speed > 10:        
            control_target_dt = 0.5 - (current_speed - 10)*0.01
        else:
            control_target_dt = 0.5
     
        control_target_distance = control_target_dt * current_speed  ## m
        if control_target_distance < 3:
            control_target_distance = 3

        ego_loc = np.array([ego_pose.position.x, ego_pose.position.y])

        trajectory_array = self.convert_trajectory_to_ndarray(trajectory)

        trajectory_dense = dense_polyline2d(trajectory_array, resolution)

        end_idx = self.get_next_idx(ego_loc, trajectory_dense, control_target_distance)
        wp_loc = trajectory_dense[end_idx]

        return wp_loc


    def _purepersuit_control(self, waypoint, ego_pose):
        """
        Estimate the steering angle of the vehicle based on the PID equations

        :param waypoint: target waypoint
        :param vehicle_transform: current transform of the vehicle
        :return: steering control in the range [-1, 1]
        """

        orientation = ego_pose.orientation
        _, _, ego_yaw = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])

        ego_x = ego_pose.position.x
        ego_y = ego_pose.position.y
        
        v_vec = np.array([math.cos(ego_yaw),
                          math.sin(ego_yaw),
                          0.0])

        target_x = waypoint[0]
        target_y = waypoint[1]

        w_vec = np.array([target_x-
                          ego_x, target_y -
                          ego_y, 0.0])

        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) /
                         (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))

        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0

        lf = 1.2
        lr = 1.65
        lwb = lf + lr
        
        v_rear_x = ego_x - v_vec[0]*lr/np.linalg.norm(v_vec)
        v_rear_y = ego_y - v_vec[1]*lr/np.linalg.norm(v_vec)
        l = (target_x-v_rear_x)*(target_x-v_rear_x)+(target_y-v_rear_y)*(target_y-v_rear_y)
        l = math.sqrt(l)

        theta = np.arctan(2*np.sin(_dot)*lwb/l)
        

        k = 1 # XXX: np.pi/180*50
        theta = theta * k
        
        return theta


    # TODO: Move into library
    def convert_trajectory_to_ndarray(self, trajectory):
        trajectory_array = [(pose.pose.position.x, pose.pose.position.y) for pose in trajectory.poses]
        return np.array(trajectory_array)

    def get_idx(self, loc, trajectory):
        dist = np.linalg.norm(trajectory-loc,axis=1)
        idx = np.argmin(dist)
        return idx

    def get_next_idx(self, start_loc, trajectory, distance):

        start_idx = self.get_idx(start_loc,trajectory)
        dist_list = np.cumsum(np.linalg.norm(np.diff(trajectory,axis = 0),axis = 1))
        for end_idx in range(start_idx,len(trajectory)-1):
            if dist_list[end_idx] > dist_list[start_idx] + distance:
                return end_idx

        rospy.logdebug("start_idx: %s, ego_loc: %s", str(start_idx), str(start_loc))
                                                    
class PreviewLateralController():
    """
    TODO(zhcao): Implement this
    """

    def __init__(self):
        self.m = 1800 #kg
        self.Iz = 3270 #kg*m^2
        self.ks = 16
        self.lf = 1.2 #
        self.caf = 70000 #N/rad
        self.car = 60000 #N/rad
        self.dt = 0.05

    def run_step(self, waypoint, EnvironmentInfo):
        """
        Execute one step of lateral control to steer the vehicle towards a certain waypoin.

        :param waypoint: target waypoint
        :return: steering control in the range [-1, 1] where:
            -1 represent maximum steering to left
            +1 maximum steering to right
        """
        return self._preview_control(waypoint, EnvironmentInfo)

    def _preview_control(self, waypoint, EnvironmentInfo):
        """
        Estimate the steering angle of the vehicle based on the preview control

        :param waypoint: target waypoint
        :param vehicle_transform: current transform of the vehicle
        :return: steering control in the range [-1, 1]
        """

        vehicle_transform = EnvironmentInfo.ego_vehicle_transform
        v_begin = vehicle_transform.location
        v_end = v_begin + carla.Location(x=math.cos(math.radians(vehicle_transform.rotation.yaw)),
                                         y=math.sin(math.radians(vehicle_transform.rotation.yaw)))

        v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])

        target_x = waypoint.x
        target_y = waypoint.y

        w_vec = np.array([target_x-
                          v_begin.x, target_y -
                          v_begin.y, 0.0])
        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) /
                         (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))

        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0

        lf = 1.2
        lr = 1.65
        lwb = lf+lr
        
        v_rear_x = v_begin.x - v_vec[0]*lr/np.linalg.norm(v_vec)
        v_rear_y = v_begin.y - v_vec[1]*lr/np.linalg.norm(v_vec)
        l = (target_x-v_rear_x)*(target_x-v_rear_x)+(target_y-v_rear_y)*(target_y-v_rear_y)
        l = math.sqrt(l)

        theta = np.arctan(2*np.sin(_dot)*lwb/l)
        

        k = 1# np.pi/180*50
        theta = theta*k
        return theta

