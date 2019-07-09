# This message describes a multi-lane dynamic map model

# ----------- Ego Vehicle Info ----------
# TODO: reconstruct vehicle state. This can be reused by junction map state
geometry_msgs/Pose ego_vehicle_pose
float32 ego_vehicle_speed

# True if the ego vehicle in a junction
# or something unexpected happened
bool in_junction # = True

# distance before next junction
float32 distance_to_junction

# -------- Multi-lanes info --------
# all lane info
LaneState[] lanes

# the lane that ego vehicle drives on
int8 ego_lane_index

# target lane index (0 refer to right-most lane in right-side driving situation)
int8 target_lane_index
