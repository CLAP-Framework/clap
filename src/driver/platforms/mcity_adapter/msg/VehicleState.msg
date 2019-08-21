float64 timestamp

#RTK state
string 	RTK_state_string

int64   RTK_seq_num
int64   RTK_timestamp_sec
int64   RTK_timestamp_nsec

int16   RTK_gps_status	
int16   RTK_gps_service	

#RTK position
float64 RTK_gps_longitude
float64 RTK_gps_latitude
float64 RTK_gps_altitude

float64 RTK_gps_UTM_x
float64 RTK_gps_UTM_y
float64 RTK_gps_UTM_z

float64 RTK_imu_quaternion_x
float64 RTK_imu_quaternion_y
float64 RTK_imu_quaternion_z
float64 RTK_imu_quaternion_w

float64 RTK_heading
float64 RTK_attitude
float64 RTK_bank

#RTK ENU speed
float64 RTK_linear_ENU_vx
float64 RTK_linear_ENU_vy
float64 RTK_linear_ENU_vz


#RTK local speed and acc
float64 RTK_gps_twist_linear_vx
float64 RTK_gps_twist_linear_vy
float64 RTK_gps_twist_linear_vz

float64 RTK_imu_linear_acc_x
float64 RTK_imu_linear_acc_y
float64 RTK_imu_linear_acc_z

#RTK local angular speed
float64 RTK_gps_twist_angular_vx
float64 RTK_gps_twist_angular_vy
float64 RTK_gps_twist_angular_vz


 
#key vehicle state
float64 x
float64 y
float64 z

float64 speed_x
float64 speed_y
float64 speed_z

float64 acc_x
float64 acc_y
float64 acc_z

float64 heading
float64 yaw_rate

#by-wire
bool 	by_wire_enabled

float64 throttle_cmd
float64 throttle_input
float64 throttle_state
bool    throttle_enabled
bool    throttle_override
bool    throttle_driver
bool    throttle_timeout
 

float64 brake_cmd
float64 brake_input
float64 brake_state
float64 brake_torq_cmd
float64 brake_torq_input
float64 brake_torq_state
bool    brake_boo_output
bool    brake_enabled
bool    brake_override
bool    brake_driver
bool    brake_timeout


float64 steer_cmd
float64 steer_state
float64 steer_torque
bool    steer_enabled
bool    steer_override
bool    steer_timeout

int16   gear_pos

float64 wheel_v_front_left
float64 wheel_v_front_right
float64 wheel_v_rear_left
float64 wheel_v_rear_right