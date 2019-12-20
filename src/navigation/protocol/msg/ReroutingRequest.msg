# This message represent request for rerouting.

# The reason for rerouting request
uint8 reason

uint8 REASON_NONE = 0
uint8 REASON_TRIP_FINISHED = 1 # current trip has been completed
uint8 REASON_MANUAL_SPECIFIED = 2 # new target specified by user

# The destination for the rerouting request
sensor_msgs/NavSatFix destination
