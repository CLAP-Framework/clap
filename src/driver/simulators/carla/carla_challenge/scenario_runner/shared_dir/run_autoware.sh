#!/bin/bash

set -e

# Default settings
CUDA="on"
IMAGE_NAME="autoware/autoware1.12.0-melodic-cuda"  
TAG_PREFIX="motionplanning-v5"
BASE_ONLY="true"
USER_ID="$(id -u)"

REL_PARENT_PATH=`dirname $0`
# ABS_PARENT_PATH=$(cd $REL_PARENT_PATH; pwd)
# WORKSPACE_HOST_DIR=$(cd `dirname $ABS_PARENT_PATH`; pwd)
WORKSPACE_HOST_DIR=$(cd $(dirname $(cd $REL_PARENT_PATH; pwd)); pwd)
WORKSPACE_NAME=$(basename $WORKSPACE_HOST_DIR)


function usage() {
    echo "Usage: $0 [OPTIONS]"
    echo "    -b,--base-only <WORKSPACE_HOST_DIR> If provided, run the base image only and mount the provided <WORKSPACE_HOST_DIR> folder."
    echo "                                       Default: Use workspace $WORKSPACE_HOST_DIR"
    echo "    -c,--cuda <on|off>                 Enable Cuda support in the Docker."
    echo "                                       Default: $CUDA"
    echo "    -h,--help                          Display the usage and exit."
    echo "    -i,--image <name>                  Set docker images name."
    echo "                                       Default: $IMAGE_NAME"
    echo "    -s,--skip-uid-fix                  Skip uid modification step required when host uid != 1000"
    echo "    -t,--tag-prefix <tag>              Tag prefix use for the docker images."
    echo "                                       Default: $TAG_PREFIX"
}

OPTS=`getopt --options b:c:hi:p:r:st: \
         --long base-only:,cuda:,help,image-name:,skip-uid-fix,tag-prefix: \
         --name "$0" -- "$@"`
eval set -- "$OPTS"
# Convert a relative directory path to absolute
function abspath() {
    local path=$1
    if [ ! -d $path ]; then
	exit 1
    fi
    pushd $path > /dev/null
    echo $(pwd)
    popd > /dev/null
}

while true; do
  case $1 in
    -b|--base-only)
      BASE_ONLY="true"
      WORKSPACE_HOST_DIR=$(abspath "$2")
      shift 2
      ;;
    -c|--cuda)
      param=$(echo $2 | tr '[:upper:]' '[:lower:]')
      case "${param}" in
        "on"|"off") CUDA="${param}" ;;
        *) echo "Invalid cuda option: $2"; exit 1 ;;
      esac
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    -i|--image-name)
      IMAGE_NAME="$2"
      shift 2
      ;;
    -s|--skip-uid-fix)
      USER_ID=1000
      shift 1
      ;;
    -t|--tag-prefix)
      TAG_PREFIX="$2"
      shift 2
      ;;
    --)
      if [ ! -z $2 ];
      then
        echo "Invalid parameter: $2"
        exit 1
      fi
      break
      ;;
    *)
      echo "Invalid option"
      exit 1
      ;;
  esac
done

echo "Using options:"
echo -e "\tImage name: $IMAGE_NAME"
echo -e "\tTag prefix: $TAG_PREFIX"
echo -e "\tCuda support: $CUDA"
if [ "$BASE_ONLY" == "true" ]; then
  echo -e "\tvehicle Home: $WORKSPACE_HOST_DIR"
fi
echo -e "\tUID: <$USER_ID>"

XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority-d
DOCKER_HOME=/home/autoware
DOCKER_XAUTH=$DOCKER_HOME/.Xauthority

SHARED_DOCKER_DIR=$DOCKER_HOME/shared_dir
SHARED_HOST_DIR=$HOME/shared_dir


WORKSPACE_DOCKER_DIR=$DOCKER_HOME/$WORKSPACE_NAME

VOLUMES="--volume=$XSOCK:$XSOCK:rw
         --volume=$XAUTH:$DOCKER_XAUTH:rw
         --volume=$SHARED_HOST_DIR:$SHARED_DOCKER_DIR:rw"

if [ "$BASE_ONLY" == "true" ]; then
    VOLUMES="$VOLUMES --volume=$WORKSPACE_HOST_DIR:$WORKSPACE_DOCKER_DIR:rw "
fi

RUNTIME=""
if [ $CUDA == "on" ]; then
    RUNTIME="--runtime=nvidia"
fi

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
    -it --rm \
    -e DISPLAY=$display \
    -e DOCKER_USER=$USER \
    -e USER=$USER \
    -e DOCKER_USER_ID=$USER_ID \
    -e DOCKER_GRP="$GRP" \
    -e DOCKER_GRP_ID=$GRP_ID \
    -e DOCKER_IMG=$IMG \
    -e USE_GPU=$USE_GPU \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,video,utility \
    -v /dev:/dev \
    -v /lib/modules:/lib/modules \
    $VOLUMES \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    --env="USER_ID=$USER_ID" \
    --privileged \
    --net=host \
    $RUNTIME \
    $IMAGE
