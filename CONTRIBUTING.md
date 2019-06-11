# Package structure
A package contains several essential folders:

- **lib**: code for library sources (*.cpp, *.py, *.pyx)
- **include**: code for headers (*.hpp, *.pyd, *.pyi)
- **msg**: message definitions (*.msg)
- **action**: action definitions (*.action)
- **srv**: service definitions (*.srv)
- **launch**: launch files (*.launch)
- **test**: unit test files
- **nodes**: code for executable sources (*.c, *.cpp, *.py, *.pyx)

# Protocol strategy
Protocal is defined in its source module. Protocal are defined based on [common_msgs](https://github.com/ros/common_msgs) of ROS and zzz protocals. Please try to prevent depend external dependencies.

# References
- [ROS CMakeLists.txt documentation](http://wiki.ros.org/catkin/CMakeLists.txt)
