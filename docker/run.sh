#!/bin/bash

xhost +

PWD=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
SRC_DIR=$PWD/../

CCACHE_DIR=${HOME}/.ccache
mkdir -p "${CCACHE_DIR}"

docker run -it --rm --privileged \
	--user="$(id -u):$(id -g)" \
	--workdir=${SRC_DIR} \
	--volume=${CCACHE_DIR}:${CCACHE_DIR}:rw \
	--volume=${SRC_DIR}:${SRC_DIR}:rw \
	--volume=/tmp/.X11-unix:/tmp/.X11-unix:ro \
	--volume=/dev/:/dev \
	--env=DISPLAY=${DISPLAY} \
	--group-add=dialout \
	--privileged \
	--name=px4-dev-simulation-noble px4io/px4-dev-simulation-noble /bin/bash -c "$1 $2 $3 $4"
