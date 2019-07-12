# This message describe a free-space driving model

# -------- Reference Path info ----------
# TODO: Do we need so much information about it? Or do we just need the path and obstacles 
LaneState reference_path

# -------- Other constaints ----------
geometry_msgs/Polygon drivable_area
RoadObstacle[] obstacles
