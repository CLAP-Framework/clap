# Defines a box-shaped 3D tracking result.

# Describe several classification result for the object
# This field is required to be sorted in descending order of scores
ObjectClass[] classes

# Current 3D bounding box.
BoundingBox bbox

# Estimated 3D velocity and accelaration
geometry_msgs/TwistWithCovariance twist
geometry_msgs/AccelWithCovariance accel

# Unique id to determine the identification of tracked object.
# This field could be a increasing number from zero or random hash number
uint64 uid

# This field is for store auxiliary text or data
string comments
