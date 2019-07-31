# How to run CARLA on a remote server
```bash
# Setup ros env and set IP
. /opt/ros/melodic/setup.sh
export ROS_MASTER_URI=http://172.16.0.1:11311
export ROS_IP=172.16.0.1

roscore

# Start CARLA docker
docker run -p 2000-2002:2000-2002 --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=0 carlasim/carla

# Spawn actors
conda activate carla # with carla PythonAPI installed
python /home/jacobz/carla_server/spawn_agent.py

# Start ROS core and ros bridge
export PYTHONPATH=$PYTHONPATH:/home/jacobz/carla_server/carla-0.9.5-py2.7-linux-x86_64.egg
roslaunch /home/jacobz/carla_server/bridge.launch
```
