# This message represents a homogeneous representation of dynamic map

Header header

# ----- Map Model -----
uint8 model

uint8 MODEL_JUNCTION_MAP = 0
uint8 MODEL_MULTILANE_MAP = 1

# ----- Map State Instances -----
JunctionMapState jmap
MultiLaneMapState mmap
