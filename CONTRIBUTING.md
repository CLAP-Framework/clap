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

# Protocol strategy
Protocal is defined in its source module. Protocal are defined based on [common_msgs](https://github.com/ros/common_msgs) of ROS and zzz protocals. Please try to prevent depend external dependencies. The protocal should contain as much information as needed. If certain module doesn't need or provide part of the data, then it should directly ignore it.

# References
- [ROS CMakeLists.txt documentation](http://wiki.ros.org/catkin/CMakeLists.txt)
- [How to structure a Python-based ROS package](http://www.artificialhumancompanions.com/structure-python-based-ros-package/)
