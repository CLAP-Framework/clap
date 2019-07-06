# This message represent a state of lane in dynamic map

# Static properties of the lane
zzz_navigation_msgs/PolyLane map_lane

# ----- Lane End Behavior -----
# distance to stop line
float32 stop_distance # = Inf
uint8 stop_state

uint8 STOP_STATE_UNKNOWN = 0
uint8 STOP_STATE_CLEAR = 1 # e.g. green light
uint8 STOP_STATE_YIELD = 2 # e.g. yellow light
uint8 STOP_STATE_YIELD_STOP = 3 # e.g. flash red light or stop sign
uint8 STOP_STATE_STOP = 4 # e.g. red light

# ----- Obstacles in Lane -----
# the front vehicle on this lane for ego vehicle
bool have_front_vehicle # = False
RoadObstacle front_vehicle

# the rear vehicle on this lane for rear vehicle
bool have_rear_vehicle # = False
RoadObstacle rear_vehicle
