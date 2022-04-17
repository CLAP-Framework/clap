# Usage
    1. Install ROS:
        a. Use python2 in system default
             
    2. Download and setup ZZZ Platform:
        a.git clone https://gitlab.com/umvdl/zzz/zzz.git -b dev/zhcao/master
        b.--recursive (clone dependency)
        c.cd zzz
        d.pip install -r requirements.txt
        e.rosdep install --from-paths src --ignore-src --rosdistro $DISTRO -y
        f.catin_make
        # if use map:
            g.apt get SUMO (check from webset)
            h.SUMO_HOME =/usr/share/sumo
            i.export PYTHONPATH=/usr/share/sumo/tools:PAYTHONPATH
             
    3. Download Carla Simulator (Recommand 0.9.11):
        a. Download from http://carla.org/2020/12/22/release-0.9.11/
         
    4. Setup Python Environment with CARLA 
        a. open zzz/export_path.sh
        b. revise the CARLA_ROOT/CARLA_VERSION/PYTHONPATH to match the local environment
        example:
            export CARLA_ROOT = /home/carla/Carla/CARLA_0.9.11
            export CARLA_VERSION = 0.9.11
            export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla$CARLA_ROOT/PythonAPI/carla/dist/carla-$CARLA_VERSION-py2.7-linux-x86_64.egg
             
    5. Quick Start:
        # Run CARLA
        - Terminal 1
            > ./CarlaUE4.sh -fps=20
        - Terminal 2 (DEMO Map)
            > python PythonAPI/utils/config.py -m Town05

        # Run ROS_Bridge
        - Terminal 3
            > bash bridge_intersection.bash

        # Run ZZZ
        - Terminal 4
            > bash main.bash

        # Visualization
        - Terminal 5
            > rviz
