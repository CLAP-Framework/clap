# CLAP系统快速操作指南



## 系统环境搭建

1. 安装Ubuntu 18.04， 磁盘500G+， 内存16G+。
2. 安装ros， 按照http://wiki.ros.org/melodic/Installation/Ubuntu的步骤安装ros-melodic-desktop-full。
3. 安装cmake，下载https://github.com/Kitware/CMake/releases/download/v3.15.4/cmake-3.15.4-Linux-x86_64.sh ，打开terminal， 执行命令 mkdir -P ~/bin 安装运行 ./cmake-3.15.4-Linux-x86_64.sh， 按照提示将cmake装到~/bin， 而后执行 echo "export PATH=$HOME/bin:$PATH" >> ~/.bashrc && source ~/.bashrc
4. 安装CUDA 10.0, https://developer.nvidia.com/cuda-10.0-download-archive?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1804, 下载runfile（local）安装及设置相应设置环境变量， 按照默认安装提示启用cuda进行安装；详细请参考https://docs.nvidia.com/cuda/archive/10.0/
5. 安装cuDNN 7.6.5（匹配cuda10.0），https://developer.nvidia.com/rdp/cudnn-archive， 下载cuDNN Library for Linux， 解压之后复制安装，详细参考https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html（[2.3.1. Installing From A Tar File](https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html#installlinux-tar)）
6. 安装tensorRT, https://developer.nvidia.com/nvidia-tensorrt-7x-download, 下载Tar File Install Packages For Linux x86文件 （TensorRT 7.0.0.11 for Ubuntu 18.04 and CUDA 10.0 tar package），直接解压复制到/opt/TensorRT-7.0.0.11， 打开terminal 执行 echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/TensorRT-7.0.0.11/lib" >> ~/.bashrc && source ~/.bashrc 详见https://docs.nvidia.com/deeplearning/tensorrt/archives/tensorrt-700/tensorrt-install-guide/index.html 
7. 检查python pip等版本，打开terminal 运行python查看是否有python2.7.*信息，输入pip --version查看版本信息， 如果不是python2或者pip2, 可以sudo unlink /usr/bin/python, 然后sudo ln -s /usr/bin/python2 /usr/bin/python; pip2切换3与python操作方式雷同；
8. 提示一点， 系统环境配好后，轻易不要update！



## CLAP源码下载及编译

1.  git clone https://gitlab.com/umvdl/zzz/zzz.git -b dev/zhcao/xiaopeng

2.  echo "export ZZZ_ROOT=$(pwd -P)" >> ~/.bashrc && source ~/.bashrc

3.  cd zzz && sudo -H pip2 install -r requirement.txt

4. ./build_zzz.sh，  注意 如果只想调试规划模块（不跑感知或者根本就没有GPU显卡）: 可以运行如下命令 rm -rf src/perception/lidar_cnn_seg_detect_trt src/perception/lidar_point_pillars_detect_trt src/perception/global_ukf_track src/tools/libtrt src/tools/tensorrt_cpp)

5. 运行模型量化脚本进行模型优化，bash scripts/env.sh && bash scripts/cnnseg_trt_builder.sh（如果不调试感知这步可忽略）

​     

## CLAP系统xiaopeng-G1启动

0. 通常clap系统部署在xiaopeng G1工控机的home目录下,  进入工控机ubuntu(user icv, pass 123456)后, cd ~/zzz;
1. 打开terminal, 执行roscore
2. 加载传感器个节点, Ctl+Shift+T另起tab, 执行./load_sensors.sh
3. 加载全局导航参考路径, Ctl+Shift+T, 执行./load_ref_path.sh
4. 加载感知+规划+控制, 执行./load_main.sh
5. 加载rviz可视化界面, 执行./load_rviz.sh
6. 加载bag capture工具, 执行./load_capture.sh
7. 最后使用xiaopengG1车钥匙即可开启autopilot功能(操作方法:中间件连续按5秒, 而后第一个键(位置靠近小鹏logo) 连按2下即可), 如不出意外车子就可以自己走了!



## CLAP系统离线Bag调试

开始...

0. 打开terminal， 执行 roscore
1. 加载全局reference-path， 另起tab 运行./load_ref_path.sh

#### 调试规划模块

2. 回放bag包， ctl+shift+T另启一个tab，执行./bagreplay.sh bagfile offset_time(跳过秒数开始play)，回放的数据包含ego-pose，image， pointscloud, objects_tracked等信息；

3. 运行 planning， 另起tab 运行 ./load_main_planning.sh

#### 调试感知+规划模块

2. 回放bag包， ctl+shift+T另启一个tab，执行./bagreplay_perception.sh bagfile offset_time(跳过秒数开始play)，回放的数据包含ego-pose，image， pointscloud等；
3. 运行感知+规划， 另起tab 运行 ./load_main.sh

最后...

4. 加载rviz可视化， 运行./load_rviz.sh

5. rviz打开后， 修改设置Displays->Global Options->Fixed Frame 将 'rslidar' 改成 'map'; 即可看到运行效果；

   

**如果想快速查看查看问题在车上的表现效果，可直接运行rosbag play bagfile + load_rviz.sh；** 



示例BAG， 百度网盘: 
地址：https://pan.baidu.com/s/1846ZN8GzlhuyUD5FQEf_jw
提取码：18xx
其中Record.txt有对bag内容的大致描述


## CLAP系统对接Carla仿真

...



## Carla with ros bridge
1. Run docker or local binary. Export port to 2000, 2001
1. Start roscore separately
1. Go to ros bridge repo, source `devel/setup.bash` and launch `src/driver/simulators/carla/carla_adapter/launch/server.launch`
1. Go to this repo, source `devel/setup.bash` and launch `src/driver/simulators/carla/carla_adapter/scripts/use_bridge/main.launch`

## Carla with scenario_runner
1. Run docker or local binary. Export port to 2000, 2001
1. Start roscore separately
1. Set `TEAM_CODE_ROOT` to `src/driver/simulators/carla/carla_adapter/scripts/use_srunner`
1. Run srunner command: `python ${ROOT_SCENARIO_RUNNER}/srunner/challenge/challenge_evaluator_routes.py --scenarios=${ROOT_SCENARIO_RUNNER}/srunner/challenge/all_towns_traffic_scenarios1_3_4.json --agent=${TEAM_CODE_ROOT}/ZZZAgent.py`

> See `zzz/src/driver/simulators/carla/carla_adapter/scripts/server.tmuxp.yaml` for details



