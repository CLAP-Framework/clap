# The size of (in meters) the bounding box surrounding the object's center pose.

# The unique numeric classification ID of object detected
# TODO(zyxin): Construct detail table of the classes, should be hierarchical. This could be a completely new message
int32 classid

# The probability or confidence value of the detected object. By convention, this value should lie in the range 0~1.
float32 score

# Other information about the class (e.g. class name). Only for debug
string comments
