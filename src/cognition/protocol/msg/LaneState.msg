# This message represent a state of lane in dynamic map

# Static properties of the lane
zzz_navigation_msgs/Lane map_lane

# distance to lane end
float32 ego_dis_to_lane_tail # = Inf
float32 ego_dis_to_lane_head # = Inf

# the front vehicles on this lane, sorted by their distances in ascending order
# TODO: Change name to front_objects
RoadObstacle[] front_vehicles

# the rear vehicles on this lane, sorted by their distances in ascending order
# TODO: Change name to rear_objects
RoadObstacle[] rear_vehicles
