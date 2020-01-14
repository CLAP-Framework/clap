export PYTHONPATH=$PYTHONPATH:/home/jacobz/Carla/PythonAPI_0.9.6/carla:/home/jacobz/Carla/PythonAPI_0.9.6/carla/dist/carla-0.9.6-py2.7-linux-x86_64.egg
cd ~/ZZZ
source devel/setup.bash
tmux && roscore 
python src/driver/simulators/carla/carla_challenge/scripts/spawn_agent
roslaunch src/driver/simulators/carla/carla_challenge/launch/challenge.launch
roslaunch /home/jacobz/ZZZ/src/driver/simulators/carla/carla_challenge/launch/challenge_veh1.launch
