export PYTHONPATH=$PYTHONPATH:/home/jacobz/Carla/PythonAPI_0.9.5/carla:/home/jacobz/Carla/PythonAPI_0.9.5/carla/dist/carla-0.9.5-py2.7-linux-x86_64.egg
cd ~/ZZZ
tmux && roscore 
python src/driver/simulators/carla/carla_challenge/scripts/spawn_agent
roslaunch src/driver/simulators/carla/carla_challenge/launch/challenge.launch
roslaunch /home/jacobz/ZZZ/src/driver/simulators/carla/carla_challenge/launch/challenge_veh1.launch
