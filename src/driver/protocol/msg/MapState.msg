
# True if the ego vehicle in a junction
# or something unexpected happened
bool in_junction

# all lane info
LaneState[] lanes

# the lane that ego vehicle drives on
int16 ego_lane_index

# target lane index
int16 target_lane_index

# distance before next edge
float64 d_to_next_lane