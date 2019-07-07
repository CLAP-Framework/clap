# Universal vehicle dynamics command output. This message is sent in high frequency

Header header

# Percent of throttle, unit in m/s^2
float32 throttle
# Percentage of brake, unit in m/s^2
float32 brake
# Percentage of steering, unit in radian
float32 steer

# Gear shift, positive means manual transmission, negative means automatic transmission
int8 gear
int8 GEAR_NONE = 0
int8 GEAR_NEUTRAL = -1
int8 GEAR_REVERSE = -2
int8 GEAR_PARKING = -3
int8 GEAR_DRIVE = -4

# Parking brake switch
bool parking_brake
