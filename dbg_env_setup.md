# instruction for bag replay 

# planning debug
## 1. Basic environment setup:
   0) Ubuntu 18.04, apt install ros-melodic-desktop-full ( follow http://wiki.ros.org/melodic/Installation/Ubuntu )
   1) CMake 3.15.4 ( https://github.com/Kitware/CMake/releases/download/v3.15.4/cmake-3.15.4-Linux-x86_64.sh , 
      make -P ~/bin, run install cmake into ~/bin, then add 'export PATH=$HOME/bin:$PATH' to ~/.bashrc )
   2) Install CUDA 10.0 + TensorRT (download tar and extract to Install Dir:/opt/TensorRT-7.0.0.11) if enable perception debug.

## 2. prepare zzz (need python2): 
   0) git clone https://gitlab.com/umvdl/zzz/zzz.git -b dev/zhcao/xiaopeng
   1) echo "export ZZZ_ROOT=$(pwd -P)" >> ~/.bashrc && source ~/.bashrc
   2) cd zzz && sudo -H pip2 install -r requirement.txt
   3) ./build_zzz.sh 
      @mark, before build_zzz (if debug planning: rm -rf src/perception/lidar_cnn_seg_detect_trt src/perception/lidar_point_pillars_detect_trt src/perception/global_ukf_track src/tools/libtrt src/tools/tensorrt_cpp)

## 3. start debug with bag
   0) roscore
   1) ./bagreplay.sh bagfile 20 
   2) ./load_ref_path.sh
   3) ./load_main.sh
   4) ./load_rviz.sh, then change setting info : Displays->Global Options->Fixed Frame 'rslidar' to 'map';



# perception debug
## 1. Basic environment setup:
   0) Ubuntu 18.04, apt install ros-melodic-desktop-full ( follow http://wiki.ros.org/melodic/Installation/Ubuntu )
   1) CMake 3.15.4 ( https://github.com/Kitware/CMake/releases/download/v3.15.4/cmake-3.15.4-Linux-x86_64.sh , 
      make -P ~/bin, run install cmake into ~/bin, then add 'export PATH=$HOME/bin:$PATH' to ~/.bashrc )
   2) Install CUDA 10.0 + TensorRT (download tar and extract to Install Dir:/opt/TensorRT-7.0.0.11) if enable perception debug.

## 2. prepare zzz (need python2): 
   0) git clone https://gitlab.com/umvdl/zzz/zzz.git -b dev/zhcao/xiaopeng
   1) echo "export ZZZ_ROOT=$(pwd -P)" >> ~/.bashrc && source ~/.bashrc
   2) cd zzz && sudo -H pip2 install -r requirement.txt
   3) ./build_zzz.sh && bash scripts/env.sh && bash scripts/cnnseg_trt_builder.sh

## 3. start debug with bag
   0) roscore
   1) ./bagreplay_perception.sh bagfile offset_time
   2) ./load_ref_path.sh
   3) ./load_main.sh
   4) ./load_rviz.sh, then change setting info : Displays->Global Options->Fixed Frame 'rslidar' to 'map';

## one more thing...
just for reproduce scenario that bug happens before, rosbag play bagfile + ./load_rviz.sh.


