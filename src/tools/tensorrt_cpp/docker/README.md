## arm交叉编译工具使用说明

| verison | comment                                | author | date       |
| ------- | -------------------------------------- | ------ | ---------- |
| V0.1    | 加入gcc、opencv、ros、cmake支持        | 赵赛   | 2020.04.02 |
|         | 加入启动脚本，镜像上传至公司私有Harbor | 赵赛   | 2020.04.13 |
|         | 加入cuda、cudnn、TensorRT支持          | 赵赛   | 2020.04.15 |
<!-- TOC -->

- [arm交叉编译工具使用说明](#arm交叉编译工具使用说明)
    - [一、概述](#一概述)
    - [二、硬件平台](#二硬件平台)
    - [三、软件环境](#三软件环境)
        - [1、安装docker](#1安装docker)
            - [安装（version 19.03.8）](#安装version-19038)
                - [设置存储库](#设置存储库)
                - [安装DOCKER CE](#安装docker-ce)
        - [2、安装qemu-aarch64-static](#2安装qemu-aarch64-static)
            - [下载](#下载)
            - [安装](#安装)
        - [3、安装nvidia-docker](#3安装nvidia-docker)
        - [4、安装nvidia-container-runtime](#4安装nvidia-container-runtime)
            - [安装](#安装-1)
            - [修改docker配置](#修改docker配置)
                - [修改Systemd drop-in file](#修改systemd-drop-in-file)
                - [修改Daemon configuration file](#修改daemon-configuration-file)
    - [四、运行docker images](#四运行docker-images)
        - [1、运行quem docker](#1运行quem-docker)
        - [2、运行交叉编译工具](#2运行交叉编译工具)
    - [五、docker image中SDK介绍](#五docker-image中sdk介绍)
        - [1、SDK](#1sdk)
        - [2、download工具](#2download工具)

<!-- /TOC -->
### 一、概述

本arm交叉编译工具，主要针对NVIDIA Xavier平台下自动驾驶软件的开发和部署，避免代码直接在arm架构下编译，提供编译效率。该环境部署依赖docker、qemu-aarch64-static、nvidia-docker、nvidia-container-runtime。

### 二、硬件平台

计算机：PC或服务器（已安装docker19.03.8）

操作系统：Linux （该工具在Ubuntu18.04、16.04环境下测试，其他版本暂未测试)

### 三、软件环境

#### 1、安装docker

reference：裴宏岩 - docker_image_development_environment_user_manual.md

##### 安装（version 19.03.8）

目前docker分为社区版 docker ce 和 企业版 docker ee。如果你安装了老版本，请卸载掉

```
sudo apt-get remove docker docker-engine docker.io containerd runc
```

使用存储库安装，在新主机上首次安装Docker CE之前，需要设置Docker存储库。之后，您可以从存储库安装和更新Docker。

######  设置存储库

更新apt包索引

```
$ sudo apt-get update
```

安装包以允许通过HTTPS使用存储库：

```
sudo apt-get install \
     apt-transport-https \
     ca-certificates \
     curl \
     gnupg-agent \
     software-properties-common
```

添加Docker的官方GPG密钥：

```
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
```

通过搜索指纹的最后8个字符，确认您现在拥有指纹9DC8 5822 9FC7 DD38 854A E2D8 8D81 803C 0EBF CD88的密钥。

```
sudo apt-key fingerprint 0EBFCD88
```

然后终端会打印以下信息

```
pub   rsa4096 2017-02-22 [SCEA]
      9DC8 5822 9FC7 DD38 854A  E2D8 8D81 803C 0EBF CD88
uid           [ unknown] Docker Release (CE deb) <docker@docker.com>
sub   rsa4096 2017-02-22 [S]
```

使用以下命令设置稳定存储库。即使您还想从边缘或测试存储库安装构建，您始终需要稳定的存储库。要添加边缘或测试存储库，请在以下命令中的单词stable之后添加单词edge或test（或两者）。

- 注意 ：下面的lsb_release -cs子命令返回Ubuntu发行版的名称，例如xenial。有时，在像Linux  Mint这样的发行版中，您可能需要将$（lsb_release -cs）更改为您的父Ubuntu发行版。例如，如果您使用的是Linux Mint Rafaela，则可以使用trusty。

```
sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"
```

- 注意：从Docker 17.06开始，稳定版本也会被推送到边缘并测试存储库。

######  安装DOCKER CE

更新apt包索引。

```
sudo apt-get update
```

安装最新版本的Docker CE，或转到下一步安装特定版本：

```
sudo apt-get install docker-ce docker-ce-cli containerd.io
```

查看Docker CE 版本

```
docker -v 
```

输出

```
Docker version 19.03.8, build afacb8b7f0
```

通过运行hello-world映像验证是否正确安装了Docker CE。

```
sudo docker run hello-world
```

出现下面这个表示你安装成功：

```
Hello from Docker!
This message shows that your installation appears to be working correctly.

To generate this message, Docker took the following steps:
 1. The Docker client contacted the Docker daemon.
 2. The Docker daemon pulled the "hello-world" image from the Docker Hub.
    (amd64)
 3. The Docker daemon created a new container from that image which runs the
    executable that produces the output you are currently reading.
 4. The Docker daemon streamed that output to the Docker client, which sent it
    to your terminal.

To try something more ambitious, you can run an Ubuntu container with:
 $ docker run -it ubuntu bash

Share images, automate workflows, and more with a free Docker ID:
 https://hub.docker.com/

For more examples and ideas, visit:
 https://docs.docker.com/get-started/
```

Docker CE已安装并正在运行。已创建docker组，但未向其添加任何用户。此时需要使用sudo来运行Docker命令。将用户添加到docker用户组

```
sudo usermod -aG docker <user>
```

reference：https://docs.docker.com/engine/install/ubuntu/

#### 2、安装qemu-aarch64-static

##### 下载

```
wget https://github.com/multiarch/qemu-user-static/releases/download/v4.0.0-5/qemu-aarch64-static
如果下载较慢，可直接从onedrive下载：https://novaai.sharepoint.cn/:u:/s/ed/EVa0waEH-dxHrjwG0qkJH7QBc71l3jxmKxKPJIUk0Q6rEQ?e=zHp1k8
```

##### 安装

```
sudo cp qemu-aarch64-static /usr/bin/               //拷贝到/usr/bin路径下
sudo chmod +x /usr/bin/qemu-aarch64-static          //赋予可执行权限
```

#### 3、安装nvidia-docker

请确保已安装了NVIDIA驱动程序和Docker 19.03。

```
# Add the package repositories
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

reference：https://github.com/NVIDIA/nvidia-docker

#### 4、安装nvidia-container-runtime

##### 安装

```
sudo apt-get install nvidia-container-runtime
```

##### 修改docker配置

**Do not follow this section if you installed the nvidia-docker2 package, it already registers the runtime.**

To register the nvidia runtime, use the method below that is best suited to your environment.
You might need to merge the new argument with your existing configuration.

###### 修改Systemd drop-in file

```
sudo mkdir -p /etc/systemd/system/docker.service.d
sudo tee /etc/systemd/system/docker.service.d/override.conf <<EOF
[Service]
ExecStart=
ExecStart=/usr/bin/dockerd --host=fd:// --add-runtime=nvidia=/usr/bin/nvidia-container-runtime
EOF
sudo systemctl daemon-reload
sudo systemctl restart docker
```

###### 修改Daemon configuration file

```
sudo tee /etc/docker/daemon.json <<EOF
{
    "runtimes": {
        "nvidia": {
            "path": "/usr/bin/nvidia-container-runtime",
            "runtimeArgs": []
        }
    }
}
EOF
sudo pkill -SIGHUP dockerd
sudo systemctl daemon-reload
sudo systemctl restart docker
```

reference：https://github.com/NVIDIA/nvidia-container-runtime/

**check that the NVIDIA Container Runtime**

```
sudo dpkg --get-selections | grep nvidia

output:
libnvidia-container-tools                       install
libnvidia-container1:amd64                      install
nvidia-container-runtime                        install
nvidia-container-toolkit                        install

sudo docker info | grep nvidia

output:
Runtimes: nvidia runc
如上所示，表明NVIDIA Container Runtime安装正确。
```

### 四、运行docker images

#### 1、运行quem docker

由于此该交叉工具为arm架构的docker镜像，要想在x86架构下运行，必须先运行quem docker：

```
docker run --rm --privileged multiarch/qemu-user-static:register

output：
Setting /usr/bin/qemu-alpha-static as binfmt interpreter for alpha
Setting /usr/bin/qemu-arm-static as binfmt interpreter for arm
Setting /usr/bin/qemu-armeb-static as binfmt interpreter for armeb
Setting /usr/bin/qemu-sparc-static as binfmt interpreter for sparc
Setting /usr/bin/qemu-sparc32plus-static as binfmt interpreter for sparc32plus
Setting /usr/bin/qemu-sparc64-static as binfmt interpreter for sparc64
Setting /usr/bin/qemu-ppc-static as binfmt interpreter for ppc
Setting /usr/bin/qemu-ppc64-static as binfmt interpreter for ppc64
Setting /usr/bin/qemu-ppc64le-static as binfmt interpreter for ppc64le
Setting /usr/bin/qemu-m68k-static as binfmt interpreter for m68k
Setting /usr/bin/qemu-mips-static as binfmt interpreter for mips
Setting /usr/bin/qemu-mipsel-static as binfmt interpreter for mipsel
Setting /usr/bin/qemu-mipsn32-static as binfmt interpreter for mipsn32
Setting /usr/bin/qemu-mipsn32el-static as binfmt interpreter for mipsn32el
Setting /usr/bin/qemu-mips64-static as binfmt interpreter for mips64
Setting /usr/bin/qemu-mips64el-static as binfmt interpreter for mips64el
Setting /usr/bin/qemu-sh4-static as binfmt interpreter for sh4
Setting /usr/bin/qemu-sh4eb-static as binfmt interpreter for sh4eb
Setting /usr/bin/qemu-s390x-static as binfmt interpreter for s390x
Setting /usr/bin/qemu-aarch64-static as binfmt interpreter for aarch64
Setting /usr/bin/qemu-aarch64_be-static as binfmt interpreter for aarch64_be
Setting /usr/bin/qemu-hppa-static as binfmt interpreter for hppa
Setting /usr/bin/qemu-riscv32-static as binfmt interpreter for riscv32
Setting /usr/bin/qemu-riscv64-static as binfmt interpreter for riscv64
Setting /usr/bin/qemu-xtensa-static as binfmt interpreter for xtensa
Setting /usr/bin/qemu-xtensaeb-static as binfmt interpreter for xtensaeb
Setting /usr/bin/qemu-microblaze-static as binfmt interpreter for microblaze
Setting /usr/bin/qemu-microblazeel-static as binfmt interpreter for microblazeel
Setting /usr/bin/qemu-or1k-static as binfmt interpreter for or1k
```
#### 2、运行交叉编译工具
登录公司内网私有Harbor仓库
```
1. 修改Daemon configuration file(/etc/docker/daemon.json)
"insecure-registries": ["192.168.3.224:8083"],
"dns": ["202.106.196.115","8.8.8.8"]

2. 登录
docker login 192.168.3.224:8083

3. 输入运维同事提供的用户名、密码

```

运行arm交叉编译工具docker image

```
docker pull 192.168.3.224:8083/arm_cross_compiler/arm64v8/xavier:V1.0
./run.sh
```
**备注:** 

本工具默认会在用户家目录下新建shared_dir作为宿主机的共享路径，与docker内/home/nvidia/workspace/cross_compiler_dir路径映射，同时为了保证交叉编译工程的正常运行，需要在xavier开发板创建/home/nvidia/workspace/cross_compiler_dir路径，作为交叉编译工程的根路径。

run.sh运行脚本如下：

```
#!/bin/bash
set -e
# Default settings
IMAGE_NAME="192.168.3.224:8083/arm_cross_compiler/arm64v8/xavier"
TAG_PREFIX="V1.0"
USER_ID="$(id -u)"

REL_PARENT_PATH=`dirname $0`
WORKSPACE_HOST_DIR=$(cd $(dirname $(cd $REL_PARENT_PATH; pwd)); pwd)
# WORKSPACE_NAME=$(basename $WORKSPACE_HOST_DIR)

echo -e "\tImage name: $IMAGE_NAME"
echo -e "\tTag prefix: $TAG_PREFIX"
# echo -e "\tCuda support: $CUDA"
if [ "$BASE_ONLY" == "true" ]; then
	echo -e "\tvehicle Home: $WORKSPACE_HOST_DIR"
fi
echo -e "\tUID: <$USER_ID>"

XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority
DOCKER_HOME=/home/nvidia/workspace/cross_compiler_dir
DOCKER_XAUTH=$DOCKER_HOME/.Xauthority
SHARED_DOCKER_DIR=$DOCKER_HOME/
SHARED_HOST_DIR=$HOME/shared_dir
# WORKSPACE_DOCKER_DIR=$DOCKER_HOME/$WORKSPACE_NAME
QEMU_HOST_PATH=/usr/bin/qemu-aarch64-static
QEMU_DOCKER_PATH=/usr/bin/qemu-aarch64-static

VOLUMES="--volume=$XSOCK:$XSOCK:rw 
		 --volume=$XAUTH:$DOCKER_XAUTH:rw
		 --volume=$SHARED_HOST_DIR:$SHARED_DOCKER_DIR:rw
		 --volume=$QEMU_HOST_PATH:$QEMU_DOCKER_PATH:rw"
RUNTIME="--runtime=nvidia"
# Create the shared directory in advance to ensure it is owned by the host user
if [ -e $SHARED_HOST_DIR ]; then
	echo "there is a shared direction $SHARED_HOST_DIR"
else
	mkdir -p $SHARED_HOST_DIR
	echo "create a shared direction $SHARED_HOST_DIR"
fi

IMAGE=$IMAGE_NAME:$TAG_PREFIX
echo "Launching $IMAGE"
echo "volume: $VOLUMES"
docker run \
    -it \
    -e DISPLAY=$display \
    -e DOCKER_USER=$USER \
    -e USER=$USER \
    -e DOCKER_USER_ID=$USER_ID \
    -e DOCKER_GRP="$GRP" \
    -e DOCKER_GRP_ID=$GRP_ID \
    -e DOCKER_IMG=$IMG \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,video,utility \
    $VOLUMES \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    --env="USER_ID=$USER_ID" \
    --privileged \
    --net=host \
    --name "armv8-xavier" \
    $RUNTIME \
    $IMAGE
    
```

### 五、docker image中SDK介绍

docker中操作系统为Ubuntu 18.04，armv8。

#### 1、SDK说明

**在docker环境中已经安装了必要的SDK工具：**

| tool     | version                     | path in docker                                               |
| -------- | --------------------------- | ------------------------------------------------------------ |
| gcc/g++  | 7.5.0                       | /usr/bin                                                     |
| python   | 2.7.17                      | /usr/bin/python                                              |
| python3  | 3.6.9                       | /usr/bin/python3                                             |
| opencv   | 3.3.1                       | include: /usr/local/include<br/>lib: /usr/local/lib/         |
| opencv   | 3.2.0                       | include: /usr/include<br>lib: /usr/lib/aarch64-linux-gnu     |
| ros      | melodic-desktop-full-1.14.5 | /opt/ros/melodic                                             |
| cmake    | 3.10.2                      | /usr/bin/cmake                                               |
| cmake    | 3.14.3                      | /opt/bin/cmake                                               |
| TensorRT | 5.1.6.1                     | include: /usr/include/aarch64-linux-gnu <br>lib: /usr/lib/aarch64-linux-gnu |
| cuda     | 10.0.326                    | include: /usr/local/cuda-10.0/include <br>lib: /usr/local/cuda-10.0/lib64 |
| cudnn    | 7.5.0.56                    | include: /usr/include <br>lib: /usr/lib/aarch64-linux-gnu    |

**cmake调用相关SDK的使用实例：**

- cmake + gcc/g++ + opencv


```
SET(CMAKE_C_COMPILER "/usr/local/bin/gcc")
SET(CMAKE_CXX_COMPILER "/usr/local/bin/g++") 
//指定gcc/g++ 7.3.0版本
//注意，设定gcc/g++路径时，一定要放在project语句之前
project( YourProjectName )

//指定opencv 3.3.1版本
set(OpenCV_DIR "/usr/local/share/OpenCV")
//指定opencv 3.2.0版本(默认)
set(OpenCV_DIR "/usr/share/OpenCV")
find_package( OpenCV REQUIRED )
```

- ros + opencv + cv_bridge

由于米文和NVIDIA Xavier平台都预装了opencv 3.3.1版本，而ros安装后又安装了opencv3.2.0。本工具内工程默认统一使用opencv3.2.0版本。如果需要使用opencv3.3.1版本时，可通过修改CMakeLists.txt文件，指定opencv的路径。

```
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
//指定opencv 3.3.1版本
set(OpenCV_DIR "/usr/local/share/OpenCV")
//指定opencv 3.2.0版本(默认)
set(OpenCV_DIR "/usr/share/OpenCV")
find_package(OpenCV REQUIRED)
```

- cuda + TensorRT
```
# CUDA
find_package(CUDA REQUIRED)
include(cmake/CUDA_utils.cmake)
# Set what architectures does nvcc support
set(CUDA_TARGET_ARCHS_SORTED ${CUDA_TARGET_ARCHS})
list(SORT CUDA_TARGET_ARCHS_SORTED)
CUDA_find_supported_arch_values(CUDA_targeted_archs ${CUDA_TARGET_ARCHS_SORTED})
message(STATUS "CUDA targeted archs: ${CUDA_targeted_archs}")
if (NOT CUDA_targeted_archs)
  message(FATAL_ERROR "None of the provided CUDA architectures ({${CUDA_TARGET_ARCHS}}) is supported by nvcc, use one or more of: ${CUDA_supported_archs}")
endif()
CUDA_get_gencode_args(CUDA_gencode_flags ${CUDA_targeted_archs})
message(STATUS "Generated gencode flags: ${CUDA_gencode_flags}")
# Add ptx & bin flags for cuda
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} ${CUDA_gencode_flags}")

# TensorRT
MESSAGE(STATUS "The project will built on arm platform(TRT 5.1.6.1).") 
set(TENSORRT_INCLUDE_DIR /usr/include/aarch64-linux-gnu/)
set(TENSORRT_LIBRARIES_DIR /usr/lib/aarch64-linux-gnu/)
include_directories(${TENSORRT_INCLUDE_DIR})
link_directories(${TENSORRT_LIBRARIES_DIR})
```

#### 2、download工具

**download 编译后工程到NVIDIA平台**

由于工程编译时，路径依赖问题，docker和nvidia平台内，应建一套相同的路径(/home/nvidia/workspace/cross_compiler_dir)。将编译后的工程直接download到nvidia平台的相同路径下；

**download脚本执行**

```
root@7830f54b409c:/home/nvidia# ./download.sh -h
Usage: ./download.sh [OPTIONS], copt the docker file to the NVIDIA platfrom.
    -h,--help        Display the usage and exit.
    -c, --cp         The file will be copied to the NVIDIA platfrom.
                     Para_1=local file path, Para_2=remote user name,
                     Para_3=remote ip address.
//将docker内编译后的工程文件夹catkin_ws下载到米文平台/home/nvidia路径下
//参数含义：需要download的文件夹、nvidia平台IP地址、nvidia平台登录用户名
root@7830f54b409c:/home/nvidia# ./download.sh -c catkin_ws/ 192.168.6.64 nvidia
nvidia@192.168.6.64's password:
catkin_make.cache                             100%  252     2.4KB/s   00:00
Makefile                                      100%   50KB 272.4KB/s   00:00
CTestTestfile.cmake                           100%  480    11.1KB/s   00:00
env.sh                                        100%  506    13.9KB/s   00:00
local_setup.zsh                               100%  293     8.5KB/s   00:00
.rosinstall                                   100%   68     1.7KB/s   00:00
local_setup.bash                              100%  283     8.1KB/s   00:00
_setup_util.py                                100%   13KB 189.5KB/s   00:00
local_setup.sh                                100%  359    11.2KB/s   00:00
... ...
```