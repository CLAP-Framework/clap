# This message is used to represent detected vehicle signals
# XXX: There's still not a usual way to do this. Is this actually needed?

uint16 flags

bool valid = 1 # indicates whether message is valid
bool left_turning_light = 2
bool right_turning_light = 4
bool brake_light = 8
