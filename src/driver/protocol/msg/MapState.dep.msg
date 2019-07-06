# TODO: separate static map from dynamic map, and this message should goes into cognition part

# ----------- Ego Vehicle Info ----------
geometry_msgs/Pose ego_vehicle_pose
float32 ego_vehicle_speed
# TODO: reconstruct 

# True if the ego vehicle in a junction
# or something unexpected happened
bool in_junction # = True

# -------- Multi-lanes info --------
# all lane info
LaneState[] lanes

# the lane that ego vehicle drives on
int16 ego_lane_index

# target lane index
int16 target_lane_index

# distance before next edge
float32 distance_to_next_lane

# -------- Reference Path info ----------
LaneState reference_path

