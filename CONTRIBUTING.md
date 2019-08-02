# Root structure
- `/docs` folder contains the overall documentation
- `/ref` folder is for importing external projects. This folder is ignored from git.
- ROS generated folders `\devel`, `\build`, `\install` are also ignored from git.

# Package structure
A package contains several essential folders:

- **cfg**: dynamically reconfigurable parameters
- **include**: code for headers (*.hpp, *.pyd, *.pyi)
- **src**: code for implementations (*.cpp, *.py, *.pyx)
- **msg**: message definitions (*.msg)
- **action**: action definitions (*.action)
- **srv**: service definitions (*.srv)
- **launch**: launch files (*.launch)
- **test**: unit test files
- **nodes**: code for executable node (*.c, *.cpp, *.py, *.pyx)
- **scripts**: code for executable sources (*.py, *.pyx)

# ROS dependency separation
In order for easier use, we will try to offer non-ROS mode. The existence of environment variable `ZZZ_ROS` determines whether the code is built upon ROS or not. To achieve this flexity, each code should have several principles:
- Majority of the functional code should be wrapped by a ROS-independent function
- A separate file provide ros wrapper for the code to work as a node
- The main implementation (includes python binding) is put into `src` and the wrapper codes should be included in sources under `nodes`

# Readme Contents
- For message definition packages: Usually related comments are contained in `.msg` files and no readme is needed.
- For algorithm implementation packages: One needs `Module IO`(input topic, output topic, parameters), `Reference`, etc.

# Protocol strategy
Protocal is defined in its source module. Protocal are defined based on [common_msgs](https://github.com/ros/common_msgs) of ROS and zzz protocals. Please try to prevent depend external dependencies. The protocal should contain as much information as needed. If certain module doesn't need or provide part of the data, then it should directly ignore it.

# Coordinate system
Global / Geographic: UTM WGS-84
Local / Ground: East-North-Up (ENU)
Body-fixed: Forward-Left-Up (FLU)

# Frames definition
Each sensor has an attached frame, and the extrinsic calibration result should be published as tf.TransformStamped between the sensors. Then the frame attached at geometry center of ego vehicle is defined as `base_link`. The `odom` frame has a coordinate originated at the start point of the vehicle (both position and orientation is fixed to), and the `map` frame has a assigned origin and the axis should be aligned with the ground coordinate (ENU).

So the transform between sensors and between `map` and `odom` are almost static, while the transform between `odom` and `base_link` are frequently updated with `nav_msgs/Odometry` messages from perception module. Transform between `map` and `odom` will being compensated by a perception or navigation module regularly. If needed, the sensor transform could also be adjusted online regularly.

# Namespaces

All node name, topic name, param name and frame name should be relative by default (that means the name is not a absolute name with specified namespace). All the namespace should be managed by roslaunch file.

# Other principles
- Lane operations should be performed in s-coordinate (one-dimensional coordinate along a curve)

# References
- [ROS CMakeLists.txt documentation](http://wiki.ros.org/catkin/CMakeLists.txt)
- [How to structure a Python-based ROS package](http://www.artificialhumancompanions.com/structure-python-based-ros-package/)
