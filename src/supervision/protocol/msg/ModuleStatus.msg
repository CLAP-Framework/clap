# This message is used to report module status. This message is only reported when changes happen.

Header header

uint8 category

uint8 CATEGORY_PERCEPTION_LIDAR = 1
uint8 CATEGORY_PERCEPTION_CAMERA = 2

string method # This field is used to indicate which method is used under thie category

int8 status
int8 STATUS_DISABLED = 0
int8 STATUS_ENABLED = 1
int8 STATUS_FINISHED = 2
int8 STATUS_BOOTING = 3
int8 STATUS_CRASHED = -1
