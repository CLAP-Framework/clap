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