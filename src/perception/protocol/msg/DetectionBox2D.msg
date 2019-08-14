# Defines a box-shaped 2D detection result.

# ----------------- Data -----------------

# The 2D data that generated these results (i.e. region proposal cropped out of
#   the image). Not required for all use cases, so it may be empty.
sensor_msgs/Image source_img

# Describe the projection approach of image with regard to the real world
# TODO: Use frame_id to represent the property of the camera frame?
uint8 projection_type

uint8 PROJECTION_BEV = 0 # Bird eye's view
uint8 PROJECTION_HEV = 1 # Human eye's view

# ---------------- Properties --------------------

# Describe several classification result for the object
# This field is required to be sorted in descending order of scores
ObjectClass[] classes

# This field indicates visual (or sound?) signal from the object
ObjectSignals signal

# 2D bounding box surrounding the object.
BoundingBox2D bbox

# This field is for store auxiliary text or data
string comments
