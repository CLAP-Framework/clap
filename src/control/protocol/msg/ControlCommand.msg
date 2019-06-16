# Universal command output

Header header

# Percent of throttle, unit in m/s^2
float32 throttle
# Percentage of brake, unit in m/s^2
float32 brake
# Percentage of steering, unit in radian
float32 steer

# Gear shift, positive means manual transmission, negative means automatic transmission
int32 gear
int32 GEAR_NONE = 0
int32 GEAR_NEUTRAL = -1
int32 GEAR_REVERSE = -2
int32 GEAR_PARKING = -3
int32 GEAR_DRIVE = -4

# Parking brake switch
bool parking_brake

# Turning signal output
uint8 turning_light
uint8 TURNING_LIGHT_OFF=0
uint8 TURNING_LIGHT_LEFT=1
uint8 TURNING_LIGHT_RIGHT=2
