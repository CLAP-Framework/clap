
# True if the ego vehicle in a junction
# or something unexpected happened
bool in_junction = True

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
# Reference Path Buffer
nav_msgs/Path reference_path_buffer

# front vehicle on reference path
bool have_front_vehicle_on_reference_path = False
ObstacleData front_vehicle_on_reference_path


# -------- Traffic Rule ----------
# speed limit
float32 speed_limit = 30

# traffic light
float32 distance_to_traffic_light = Inf
bool traffic_light_state = True # green

# stop sign
float32 distance_to_stop_sign = Inf


