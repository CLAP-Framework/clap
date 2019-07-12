# This message represents a homogeneous representation of dynamic map

Header header

# Ego Vehicle Info
zzz_driver_msgs/RigidBodyState ego_state

# ----- Map Model indicator -----
# Junction map is given all the time, while multi-lane map
# and other maps are given when the road is properly structured
uint8 model

uint8 MODEL_JUNCTION_MAP = 0
uint8 MODEL_MULTILANE_MAP = 1

# Map State Instances
JunctionMapState jmap
MultiLaneMapState mmap
