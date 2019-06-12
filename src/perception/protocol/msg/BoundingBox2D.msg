# A 2D bounding box that can be rotated about its center. All dimensions are in pixels, but represented using floating-point

# The position and orientation of the rigid body center
geometry_msgs/Pose2D pose

# The size (in meters) of the bounding box surrounding the object relative to the pose of its center.
Dimension2D dimension