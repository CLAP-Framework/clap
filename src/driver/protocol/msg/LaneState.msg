
# the rightest lane is 0
int16 index

# Central path
nav_msgs/Path central_path

# the front vehicle on this lane for ego vehicle
# TODO: create custom message
bool have_front_vehicle = False
ObstacleData front_vehicle

# the rear vehicle on this lane for rear vehicle
bool rear_vehicle = False
ObstacleData rear_vehicle


