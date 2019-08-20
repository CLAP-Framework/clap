import numpy as np
from geometry_msgs.msg import AccelWithCovariance
from nav_msgs.msg import Odometry
from zzz_driver_msgs.msg import RigidBodyState, RigidBodyStateStamped

import tf.transformations as tft
import tf2_ros as tf2

# TODO(zyxin): covariance propagation can be preserved use a new library which preserve covariance matrix for vector operation
#   this could be written by us, but is there any existing one?
# TODO: Let this function support various input: pose + pose, pose + state, state + state, pose + odom, etc.

def get_absolute_state(relative_state, base_state, check_frame=True):
    '''
    Calculate absolute rigid body state.
    
    Parameters:
    relative_state ('rel' relative to 'base'): state in relative coordinate
        be in type RigidBodyStateStamped
    base_state ('base' relative to 'static'): state of the relative coordinate,
        be in type RigidBodyStateStamped or Odometry

    Return:
    absolute_state ('rel' relative to 'static'): be in type RigidBodyStateStamped
    '''
    if type(relative_state) == RigidBodyStateStamped:
        rel_pose  = relative_state.state.pose
        rel_twist = relative_state.state.twist
        rel_accel = relative_state.state.accel
        rel_header = relative_state.header
        rel_child_frame = relative_state.state.child_frame_id
    if type(base_state) == RigidBodyStateStamped:
        base_pose  = base_state.state.pose
        base_twist = base_state.state.twist
        base_accel = base_state.state.accel
        base_header = base_state.header
        base_child_frame = base_state.state.child_frame_id
    elif type(base_state) == Odometry:
        base_pose  = base_state.pose
        base_twist = base_state.twist
        base_accel = AccelWithCovariance()
        base_header = base_state.header
        base_child_frame = base_state.child_frame_id

    state = RigidBodyStateStamped()
    state.header.stamp = rel_header.stamp
    state.header.frame_id = base_header.frame_id
    if check_frame:
        assert rel_header.frame_id == base_child_frame
    state.state.child_frame_id = rel_child_frame

    # phi_abs = phi_base + phi_rel (phi: orientation vector)
    q_base = [base_pose.pose.orientation.x, base_pose.pose.orientation.y, base_pose.pose.orientation.z, base_pose.pose.orientation.w]
    q_rel = [rel_pose.pose.orientation.x, rel_pose.pose.orientation.y, rel_pose.pose.orientation.z, rel_pose.pose.orientation.w]
    q_abs = tft.quaternion_multiply(q_base, q_rel)
    R_base = tft.quaternion_matrix(q_base)[:3,:3]
    state.state.pose.pose.orientation.x = q_abs[0]
    state.state.pose.pose.orientation.y = q_abs[1]
    state.state.pose.pose.orientation.z = q_abs[2]
    state.state.pose.pose.orientation.w = q_abs[3]

    # r_abs = r_rel + r_base (r: position vector)
    t_base = np.array([base_pose.pose.position.x, base_pose.pose.position.y, base_pose.pose.position.z])
    t_rel = np.array([rel_pose.pose.position.x, rel_pose.pose.position.y, rel_pose.pose.position.z])
    t_rel = t_rel.dot(R_base.T) # convert to representation in static frame
    t_abs = t_rel + t_base
    state.state.pose.pose.position.x = t_abs[0]
    state.state.pose.pose.position.y = t_abs[1]
    state.state.pose.pose.position.z = t_abs[2]

    # omega_abs = omega_rel + omega_base (omega: angular velocity vector)
    w_base = np.array([base_twist.twist.angular.x, base_twist.twist.angular.y, base_twist.twist.angular.z])
    w_rel = np.array([rel_twist.twist.angular.x, rel_twist.twist.angular.y, rel_twist.twist.angular.z])
    w_rel = w_rel.dot(R_base.T) # convert to representation in static frame
    w_abs = w_rel + w_base
    state.state.twist.twist.angular.x = w_abs[0]
    state.state.twist.twist.angular.y = w_abs[1]
    state.state.twist.twist.angular.z = w_abs[2]

    # v_abs = (v_base + omega_rel x r_rel) + v_rel (v: linear velocity vector)
    v_base = np.array([base_twist.twist.linear.x, base_twist.twist.linear.y, base_twist.twist.linear.z])
    v_rel = np.array([rel_twist.twist.linear.x, rel_twist.twist.linear.y, rel_twist.twist.linear.z])
    v_rel = v_rel.dot(R_base.T) # convert to representation in static frame
    v_abs = v_base + np.cross(w_base, t_rel) + v_rel
    state.state.twist.twist.linear.x = v_abs[0]
    state.state.twist.twist.linear.y = v_abs[1]
    state.state.twist.twist.linear.z = v_abs[2]

    # epsilon_abs = epsilon_base + epsilon_rel + omega_base x omega_rel (epsilon: angular accleration vector)
    e_base = np.array([base_accel.accel.angular.x, base_accel.accel.angular.y, base_accel.accel.angular.z])
    e_rel = np.array([rel_accel.accel.angular.x, rel_accel.accel.angular.y, rel_accel.accel.angular.z])
    e_base = e_base.dot(R_base.T) # convert to representation in static frame
    e_abs = e_base + e_rel + np.cross(w_base, w_rel)
    state.state.accel.accel.angular.x = e_abs[0]
    state.state.accel.accel.angular.y = e_abs[1]
    state.state.accel.accel.angular.z = e_abs[2]

    # a_abs = (a_base + epsilon_base x r_rel + omega_base x (omega_base x r_rel)) + a_rel + (2omega_base x v_rel)
    # (a: linear acceleration vector)
    a_base = np.array([base_accel.accel.linear.x, base_accel.accel.linear.y, base_accel.accel.linear.z])
    a_rel = np.array([rel_accel.accel.linear.x, rel_accel.accel.linear.y, rel_accel.accel.linear.z])
    a_base = a_base.dot(R_base.T) # convert to representation in static frame
    a_abs = a_base + np.cross(e_base, t_rel) + np.cross(w_base, np.cross(w_base, t_rel)) + a_rel + 2*np.cross(w_base, v_rel)
    state.state.accel.accel.linear.x = a_abs[0]
    state.state.accel.accel.linear.y = a_abs[1]
    state.state.accel.accel.linear.z = a_abs[2]

    return state
