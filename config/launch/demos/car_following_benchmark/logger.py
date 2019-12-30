import rospy
import numpy as np
from addict import Dict as edict
from nav_msgs.msg import Odometry

global_states = edict(
    ego_states = [],
    front_states = [],
    front_states_prcp = []
)

def ego_callback(msg):
    global_states.ego_states.append((msg.header.stamp.to_sec(), msg.pose.pose.position.x, msg.pose.pose.position.y, msg.twist.twist.linear.x, msg.twist.twist.linear.y))

def front_callback(msg):
    global_states.front_states.append((msg.header.stamp.to_sec(), msg.pose.pose.position.x, msg.pose.pose.position.y, msg.twist.twist.linear.x, msg.twist.twist.linear.y))

def shutdown():
    ego_states = np.array(global_states.ego_states)
    np.savetxt("ego_states.txt", ego_states)

    front_states = np.array(global_states.front_states)
    np.savetxt("front_states.txt", front_states)

if __name__ == "__main__":
    rospy.init_node("experiment_logger")
    ego_states_subs = rospy.Subscriber("/carla/egov/odometry", Odometry, ego_callback)
    front_states_subs = rospy.Subscriber("/carla/frontv/odometry", Odometry, front_callback)
    front_states_prcp_subs = None

    rospy.on_shutdown(shutdown)
    print("Logging started")
    rospy.spin()
