# This message describe a lane with parametric representation
# XXX: This message is now actually a draft

# the rightest lane is 0, reference lane is -1
int16 index

# Speed limit (km/h)
float32 speed_limit

# Central path
float32[] central_path_coeffs

uint8 central_path_type
uint8 CENTRAL_PATH_LINE = 0
uint8 CENTRAL_PATH_BEZIER = 1