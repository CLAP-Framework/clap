<!--
 * @Author: your name
 * @Date: 2020-05-19 15:45:27
 * @LastEditTime: 2020-05-19 17:34:48
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /vehicle/src/localization/README.MD
--> 
# Coordinate system frame:

- car body frame: x-forword y-left z-up
- navigation frame: East-north-up

# publish topics


*topics:*
- /localization/gps/fix          : sensor_msgs/NavSatFix                   
- /localization/gps/nav_status   : std_msgs/String                         
- /localization/gps/odom         : nav_msgs/Odometry                       
- /localization/gps/pos_type     : std_msgs/String                         
- /localization/gps/time_ref     : sensor_msgs/TimeReference               
- /localization/gps/vel          : geometry_msgs/TwistWithCovarianceStamped
- /localization/imu/data         : sensor_msgs/Imu

# how to use

1. to get latitude-longitude-height, subscribe /localization/gps/fix
2. to get  UTM location,  subscribe   /localization/gps/odom
3. to get oriention,   subscribe  /localization/gps/odom  or /localization/imu/data 
4. to get velocity(vel_E, vel_N, vel_U),  subscribe /localization/gps/vel 
5. to get acceleration(m/s^2) and angular velocity(rad/s),   subscribe /localization/gps/imu/data


*note:  acceleration and angular velocity is in car body frame.*

