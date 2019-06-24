# Defines a box-shaped 3D detection result.

# Describe several classification result for the object
# This field is required to be sorted in descending order of scores
ObjectClass[] classes

# 3D bounding box surrounding the object.
BoundingBox bbox

# The 3D data that generated these results (i.e. object region cropped out of
#   the point cloud). This information is not required for all detectors, so it may be empty.
sensor_msgs/PointCloud2 source_cloud