# This message contains all the information that planning needs for a road object

uint64 uid

# Estimated kinematic properties
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
geometry_msgs/AccelWithCovariance accel

# Optional field to represent the physical boundary of the object
geometry_msgs/Polygon shape
# Null uncertainty of Inf uncertainty means that the shape is not actually generated
float32[] shape_uncertainty

uint8 behavior
uint8 BEHAVIOR_UNKNOWN = 0
uint8 BEHAVIOR_STOPPING = 1 # vehicle is going to stop completely or stay stopped
uint8 BEHAVIOR_FOLLOW = 2 # follow lane or straight line
uint8 BEHAVIOR_MOVING_LEFT = 3 # changing to its left lane or branch left
uint8 BEHAVIOR_MOVING_RIGHT = 4 # changing to its right lane or branch right

# This field is preserved to adapt to yield situation.
# Yield situation includes left turn, right turn, sequencial stop sign, etc.
# Yield means if your path conflict with the object's, you should wait for the object
# to go through the conflict point.
uint8 priority
uint8 PRIORITY_NORMAL = 0
uint8 PRIORITY_CAUTIOUS = 1 # You can move when this obstacle is far away
uint8 PRIORITY_STOP = 2 # Should let this vehicle to go and then you can move
