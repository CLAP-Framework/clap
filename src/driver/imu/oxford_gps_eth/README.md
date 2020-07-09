Ethernet interface to OxTS GPS receivers using the NCOM packet structure
===============================

![](RT3v2.jpg)

Tested with the RT3000v2 receiver.

### Published Topics
- `gps/fix` ([sensor_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html)) GPS position
- `gps/vel` ([geometry_msgs/TwistWithCovarianceStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html)) GPS velocity in utm frame
- `gps/odom` ([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)) GPS velocity in utm frame
- `imu/data` ([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)) Orientation, angular rates, linear accelerations

### Parameters
- `interface` Restrict to single network interface, example: `eth0`. Default `<empty>`
- `ip_address` Restrict to single ip address. Default `<empty>`
- `port` UDP listen port. Default `3000`
- `frame_id` The frame-ID for gps position and imu. Default `gps`
- `frame_id_vel` The frame-ID for gps velocity. Default `utm`

### Example usage
```
rosrun oxford_gps_eth gps_node
```
OR
```
roslaunch oxford_gps_eth gps.launch
```

### FAQ
I see ```Connected to Oxford GPS at <ip_address>:3000```, but no messages are published.  
Invalid data is not published. Move faster than 5 m/s to initialize the sensor.

