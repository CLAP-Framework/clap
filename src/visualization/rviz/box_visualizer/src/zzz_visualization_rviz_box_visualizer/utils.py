import rospy
from std_msgs.msg import ColorRGBA

def check_color(value):
    if value > 255.:
        return 1
    elif value < 0:
        return 0.
    else:
        return value / 255.

def check_alpha(value):
    if value > 1:
        return 1
    elif value < 0.1:
        return 0.1
    else:
        return value

# Colors should be in order r, g, b, a
def parse_color(in_color):
    color = ColorRGBA()
    if len(in_color) == 4:
        color.r = check_color(in_color[0])
        color.g = check_color(in_color[1])
        color.b = check_color(in_color[2])
        color.a = check_alpha(in_color[3])
    else:
        rospy.logerr("Cannot resolve color value. Check configuration please!")
    return color
