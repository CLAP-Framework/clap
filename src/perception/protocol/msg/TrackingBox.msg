# Defines a box-shaped 3D tracking result.

# Describe several classification result for the object
# This field is required to be sorted in descending order of scores
ObjectClass[] classes

# Current 3D bounding box.
BoundingBox pose

# Estimated 3D velocity
geometry_msgs/TwistWithCovariance twist
