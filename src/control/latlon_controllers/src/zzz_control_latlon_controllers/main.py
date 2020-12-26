
import rospy
import tf
import numpy as np

from zzz_driver_msgs.utils import get_speed
from zzz_control_msgs.msg import ControlCommand
from zzz_control_latlon_controllers.lateral import PurePersuitLateralController
from zzz_control_latlon_controllers.longitudinal import PIDLongitudinalController

from threading import Lock

class MainController():
    """
    PurePersuitController is the combination of a PID controller for longitudinal control and a pure persuilt controller for lateral control
    to perform the low level control a vehicle from client side
    """

    def __init__(self, lon_controller=None, lat_controller=None):
        self._lon_controller = lon_controller if lon_controller is not None else PIDLongitudinalController()
        self._lat_controller = lat_controller if lat_controller is not None else PurePersuitLateralController()
        self.ego_state = None
        self.desired_trajectory = None
        self.desired_speed = 30.0
        self._trajectory_lock = Lock()
        self._ego_state_lock = Lock()

    def update_decision(self, decision):
        with self._trajectory_lock:
            self.desired_trajectory = decision.trajectory
            self.desired_speed = decision.desired_speed[-1] #FIXME: Zwt: which point to follow?

    def update_pose(self, pose):
        with self._ego_state_lock:
            self.ego_state = pose.state
           
    def ready_for_control(self, short_distance_thres = 2, less_points_thres = 3):
        if self.desired_trajectory is None or len(self.desired_trajectory.poses) == 0:
            rospy.logdebug("Haven't recevied trajectory")
            return False

        last_loc = np.array([self.desired_trajectory.poses[-1].pose.position.x, self.desired_trajectory.poses[-1].pose.position.y]) 
        ego_loc = np.array([self.ego_state.pose.pose.position.x, self.ego_state.pose.pose.position.y])
        d = np.linalg.norm(ego_loc - last_loc)

        if d < short_distance_thres:
            rospy.loginfo("Vehicle stopped since it reached target point (dist:%.3f)", d)
            return False
        
        traj_len = len(self.desired_trajectory.poses)
        if traj_len < less_points_thres:
            rospy.loginfo("Vehicle stopped because received trajectory points are too few (traj_len:%d)", traj_len)
            return False
        
        return True

    def run_step(self):
        """
        Execute one step of control invoking both lateral and longitudinal PID controllers to track a trajectory
        at a given target_speed.
        return: control
        """
        
        if not self.ego_state or not self.ready_for_control():
            control_msg = ControlCommand()
            control_msg.accel = -1
            control_msg.steer = 0
            
            return control_msg

        rospy.logdebug("received target speed:%f, current_speed: %f", self.desired_speed, get_speed(self.ego_state))
        
        ego_pose = self.ego_state.pose.pose

        # with self._trajectory_lock:
        target_speed = self.desired_speed
        trajectory = self.desired_trajectory
        current_speed = get_speed(self.ego_state)

        accel = self._lon_controller.run_step(target_speed, current_speed)
        steer = self._lat_controller.run_step(ego_pose, trajectory, current_speed)

        rospy.logdebug("accel = %f, steer = %f ***", accel, steer)

        control_msg = ControlCommand()
        control_msg.accel = accel
        control_msg.steer = steer

        return control_msg
