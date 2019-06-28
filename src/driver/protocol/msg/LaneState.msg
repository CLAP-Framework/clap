
# the rightest lane is 0, reference lane is -1
int16 index

# Central path
nav_msgs/Path central_path

# stop sign
float32 distance_to_stop_sign_line # = Inf

# Traffic light
float32 distance_to_traffic_light_line # = Inf
uint8 traffic_light_state

uint8 TRAFFIC_LIGHT_NONE = 0
uint8 TRAFFIC_LIGHT_RED = 1
uint8 TRAFFIC_LIGHT_YELLOW = 2
uint8 TRAFFIC_LIGHT_GREEN = 3

# Speed limit  km/h 
float32 speed_limit

# the front vehicle on this lane for ego vehicle
bool have_front_vehicle # = False
ObstacleData front_vehicle

# the rear vehicle on this lane for rear vehicle
bool have_rear_vehicle # = False
ObstacleData rear_vehicle


