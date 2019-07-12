# This message contains all the information that planning needs for a road object


# Unique indentity of the obstacle
uint64 uid

# The confidence of existence, can be used to determine whether this object is valid, or invalid
# A possible way of get the confidence is by using tracking age
float32 confidence 

# Estimated kinematic properties
zzz_driver_msgs/RigidBodyState state

bool static # A flag to mark whether the object is static

# TODO: Add history trajectories

# ----- Physical Boundary (Optional) -----
uint8 shape_type
uint8 SHAPE_UNDEFINED = 0
uint8 SHAPE_POLYGON = 1

# Representation of the object if it's represented by polygon
geometry_msgs/Polygon shape
# Null uncertainty of Inf uncertainty means that the shape is not actually generated
float32[] shape_uncertainty

# ----- High level behavior estimation -----

uint8 behavior
uint8 BEHAVIOR_UNKNOWN = 0
uint8 BEHAVIOR_STOPPING = 1 # vehicle is going to stop completely or stay stopped
uint8 BEHAVIOR_FOLLOW = 2 # follow lane or straight line
uint8 BEHAVIOR_MOVING_LEFT = 3 # changing to its left lane or branch left
uint8 BEHAVIOR_MOVING_RIGHT = 4 # changing to its right lane or branch right
