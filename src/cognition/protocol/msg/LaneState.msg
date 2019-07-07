# This message represent a state of lane in dynamic map

# Static properties of the lane
zzz_navigation_msgs/Lane map_lane

# distance to lane end
float32 stop_distance # = Inf

# the front vehicle on this lane for ego vehicle
bool have_front_vehicle # = False
RoadObstacle front_vehicle

# the rear vehicle on this lane for rear vehicle
bool have_rear_vehicle # = False
RoadObstacle rear_vehicle
