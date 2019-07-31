# This message describes a traffic light detection result


# Location of traffic light.
BoundingBox2D location

# Projection type of traffic light image
uint8 projection_type

uint8 PROJECTION_BEV = 0 # Bird eye's view
uint8 PROJECTION_HEV = 1 # Human eye's view

# Traffic light state
uint8 traffic_light_state

uint8 TRAFFIC_LIGHT_UNKNOWN = 0
uint8 TRAFFIC_LIGHT_RED = 1
uint8 TRAFFIC_LIGHT_YELLOW = 2
uint8 TRAFFIC_LIGHT_GREEN = 3
