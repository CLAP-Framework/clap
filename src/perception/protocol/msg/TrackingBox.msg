# Defines a box-shaped 3D tracking result.

# Describe several classification result for the object
# This field is required to be sorted in descending order of scores
ObjectClass[] classes

# Current 3D bounding box.
BoundingBox bbox # FIXME(zyxin): Change from pose to bbox

# Estimated 3D velocity
geometry_msgs/TwistWithCovariance twist

# Unique id to determine the identification of tracked object.
# This field could be a increasing number from zero or random hash number
uint64 uid

# This field is for store auxiliary text or data
string comments
