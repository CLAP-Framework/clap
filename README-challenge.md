# This document is built to describe how to use the framework to run carla agent

1. Installation: remember to clone all the submodules
2. Installation: use `catkin_make` to build. There's a little patch in `src/driver/simulators/carla/carla_bridge/rviz_carla_plugin/CMakeLists.txt`, the QT version doesn't need to be exact.
3. Enable ros environment: `source <zzz path>/devel/setup.bash`
4. Add carla to python environment: `easy_install --user <carla path>/PythonAPI/carla/dist/<Py2.7 Egg>` and then `export PYTHONPATH=$PYTHONPATH:<carla path>/PythonAPI/carla`
5. Launch server: `roslaunch zzz_driver_simulators_carla_adapter server.launch`
5. (Optional) Publishe reference path: `python ~/Desktop/trajectory_publisher.py`. (This can be removed when the SUMO map inside the Carla MCity map is fixed)
6. Launch agent: `roslaunch <zzz path>src/driver/simulators/carla/carla_adapter/scripts/use_bridge/main.launch`