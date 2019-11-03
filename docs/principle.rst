Standard Principles
===================


File organization
#################

Root structure
**************

- `/docs` folder contains the overall documentation
- `/ref` folder is for importing external projects. This folder is ignored from git.
- ROS generated folders `\devel`, `\build`, `\install` are also ignored from git.

Package structure
*****************
A package contains several essential folders:

- **doc** : folder to store essential documentation files
- **cfg** : dynamically reconfigurable parameters
- **include** : code for headers (\*.hpp, \*.pyd, \*.pyi)
- **src** : code for implementations (\*.cpp, \*.py, \*.pyx)
- **msg** : message definitions (\*.msg)
- **action** : action definitions (\*.action)
- **srv** : service definitions (\*.srv)
- **launch** : launch files (\*.launch)
- **test** : unit test files
- **nodes** : code for executable node (\*.c, \*.cpp, \*.py, \*.pyx)
- **scripts** : code for executable sources (\*.py, \*.pyx)


Protocol strategy
*****************

Protocal is defined in its source module. Protocal are defined based on [common_msgs](https://github.com/ros/common_msgs) of ROS and zzz protocals. Please try to prevent depend external dependencies. The protocal should contain as much information as needed. If certain module doesn't need or provide part of the data, then it should directly ignore it.

Coordinate system
#################

Coordinate definition
*********************

    These definitions adhere to ROS standard `REP 103 <https://www.ros.org/reps/rep-0103.html>`_ .

- Global / Geographic: UTM WGS-84
- Local / Ground: East-North-Up (ENU)
- Body-fixed: Forward-Left-Up (FLU), origin at rear axle center


Frames definition
*****************

    These definitions adhere to ROS standard `REP 103 <https://www.ros.org/reps/rep-0103.html>`_ .

Each sensor has an attached frame, and the extrinsic calibration result should be published to `/tf` as TransformStamped between the sensors. Then some common frames are defined listed below:

- `base_link`: attached at somewhere (e.g. the geometry center) of ego vehicle body with FLU coordinate. The transform between frame of other sensors and `base_link` is usually static. (published as `tf_static`)
- `base_chassis`: attached at rear axis center of ego vehicle with FLU coordinate. Since this frame is attached under suspension, the transform between `base_link` and `base_body` is actualy dynamic, but we usually ignore it.
- `base_footprint`: attached at the projection of rear axis center on ground with projected FLU coordinate, which is a FLU coordinate projected on ground. X-Y plane of the projected coordinate is aligned with tangent plane of the road.
- `odom`: a coordinate originated at the start point of the vehicle (both position and orientation is fixed to). The transform between `base_link` and `odom` will be updated whenever a new odometry estimation is performed.
- `map`: frame with a assigned origin and the axis should be aligned with the ground coordinate (ENU). The transform between `odom` and `map` will be updated when the map area is updated or odometry drift is corrected.

So the transform between sensors and between `map` and `odom` are almost static, while the transform between `odom` and `base_link` are frequently updated with `nav_msgs/Odometry` messages from perception module. Transform between `map` and `odom` will being compensated by a perception or navigation module regularly. If needed, the sensor transform could also be adjusted online regularly.

Also note that the coordinate attached to the cameras follows [standard definition of ROS](http://wiki.ros.org/image_pipeline/CameraInfo)

ROS System
##########

ROS Namespaces
**************

All node name, topic name, param name and frame name should be relative by default (that means the name is not a absolute name with specified namespace). All the namespace should be managed by roslaunch file.

ROS Packages
************

In all the nodes, code in `lib` (for C++) and `src` (for Python) should contain codes that can be run without ROS. We will provide unified interface for logging, parameter passing and data structure generation.

Other principles
################

Lane operations should be performed in s-coordinate (one-dimensional coordinate along a curve)
