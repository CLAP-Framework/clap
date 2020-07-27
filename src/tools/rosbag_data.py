
import rosbag
import numpy as np


import cv2
from cv_bridge import CvBridge

'''
/left_usb_cam/image_raw/compressed               11865 msgs    : sensor_msgs/CompressedImage
/xp/auto_control                                 18893 msgs    : xpmotors_can_msgs/AutoCtlReq
/xp/auto_state                                   47651 msgs    : xpmotors_can_msgs/AutoState
/xp/auto_state_ex                                47660 msgs    : xpmotors_can_msgs/AutoStateEx
/xp/eps_status                                   19060 msgs    : xpmotors_can_msgs/EPSStatus
/xp/esc_status                                   19059 msgs    : xpmotors_can_msgs/ESCStatus
/zzz/cognition/ego_markerarray                   14385 msgs    : visualization_msgs/MarkerArray
/zzz/cognition/lanes_markerarray                 14385 msgs    : visualization_msgs/MarkerArray
/zzz/cognition/local_dynamic_map/map_with_ref    12753 msgs    : zzz_cognition_msgs/MapState
/zzz/cognition/obstacles_markerarray             14385 msgs    : visualization_msgs/MarkerArray
/zzz/navigation/ego_pose                        113697 msgs    : zzz_driver_msgs/RigidBodyStateStamped
/zzz/perception/objects_tracked                   6998 msgs    : zzz_perception_msgs/TrackingBoxArray
/zzz/planning/decision_trajectory                10238 msgs    : zzz_planning_msgs/DecisionTrajectory
/zzz/planning/decision_trajectory_path           10238 msgs    : nav_msgs/Path
'''


bag = rosbag.Bag('bag/0614/RLS06141614_.bag')

control_data_outfile = open("control.txt", "a")
control_data_format = " ".join(("%f",)*3)+"\n"

traffic_data_outfile = open("traffic.txt", "a")
traffic_data_format = " ".join(("%f",)*8)+"\n"

automode_data_outfile = open("automode.txt", "a")
automode_data_format = " ".join(("%f",)*2)+"\n"

surrounding_obj_data_outfile = open("surrounding_obj.txt", "a")
surrounding_obj_data_format = " ".join(("%f",)*5)+"\n"

decision_data_outfile = open("decision.txt", "a")
decision_data_format = " ".join(("%f",)*5)+"\n"

# TODO: 


for topic, msg, t in bag.read_messages(topics=['/gps/fix', '/gps/odom', '/imu/data']):
    
    if topic == '/gps/fix':
        # env vehicle info
        for vehicle in msg.jmap.obstacles:
            record_data = []
            vehicle_x = vehicle.state.pose.pose.position.x # format
            vehicle_y = vehicle.state.pose.pose.position.y
            vehicle_vx = vehicle.state.twist.twist.linear.x
            vehicle_vy = vehicle.state.twist.twist.linear.y
            
            record_data.append(t.to_sec())
            record_data.append(vehicle_x)
            record_data.append(vehicle_y)
            record_data.append(vehicle_vx)
            record_data.append(vehicle_vy)
            surrounding_obj_data_outfile.write(surrounding_obj_data_format % tuple(np.array(record_data)))

        # ego info storage
        record_data = []
        detected_obj = len(msg.jmap.obstacles)
        # obj_in_lane = 0
        # if len(msg.mmap.lanes) > 0:
        #     print(len(msg.mmap.lanes))
        #     obj_in_lane = (len(msg.mmap.lanes[0].front_vehicles) +
        #                 len(msg.mmap.lanes[1].front_vehicles) +
        #                 len(msg.mmap.lanes[0].rear_vehicles) +
        #                 len(msg.mmap.lanes[1].rear_vehicles))
        # else:
        obj_in_lane = 0
        record_data.append(t.to_sec())
        record_data.append(detected_obj)
        record_data.append(obj_in_lane)
        ego_x = msg.ego_state.pose.pose.position.x
        ego_y = msg.ego_state.pose.pose.position.y
        ego_ffstate_y = msg.mmap.ego_lane_index
        ego_vx =  msg.ego_state.twist.twist.linear.x
        ego_vy =  msg.ego_state.twist.twist.linear.y
        record_data.append(ego_x)
        record_data.append(ego_y)
        record_data.append(ego_ffstate_y)
        record_data.append(ego_vx)
        record_data.append(ego_vy)
        traffic_data_outfile.write(traffic_data_format % tuple(np.array(record_data)))

    if topic == '/xp/auto_state_ex':
        auto_drive_mode = msg.CurDriveMode
        record_data = []
        record_data.append(t.to_sec())
        record_data.append(auto_drive_mode)
        automode_data_outfile.write(automode_data_format % tuple(np.array(record_data)))

    if topic == '/xp/auto_control':
        target_speed = msg.TarSpeedReq
        Streeing_angle = msg.EPSAngleReq
        record_data = []
        record_data.append(t.to_sec())
        record_data.append(target_speed)
        record_data.append(Streeing_angle)
        control_data_outfile.write(control_data_format % tuple(np.array(record_data)))

    if topic == '/zzz/planning/decision_trajectory':
        for point in msg.trajectory.poses:
            record_data = []
            record_data.append(t.to_sec())
            record_data.append(msg.desired_speed)
            record_data.append(msg.RLS_action) # format ? 
            record_data.append(point.pose.position.x)
            record_data.append(point.pose.position.y)
            decision_data_outfile.write(decision_data_format % tuple(np.array(record_data)))
            
    # if topic == '/left_usb_cam/image_raw/compressed':
    #     bridge = CvBridge()
    #     cv_image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
    #     cv2.imshow("Image window", cv_image)
    #     cv2.waitKey(3)

            

