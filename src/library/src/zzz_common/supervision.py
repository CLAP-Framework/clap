import rospy
from zzz_supervision_msgs.msg import ModuleStatus

status_enum = ModuleStatus

def publish_module_status(category, method, status):
    msg = ModuleStatus()
    msg.category = category
    msg.method = method
    msg.status = status
    rospy.Publisher("/module_status", ModuleStatus, queue_size=1, latch=True).publish(msg)

def publish_module_beat():
    raise NotImplementedError()
