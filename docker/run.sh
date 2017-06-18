#!/bin/sh

USER_UID=$(id -u)
USER_GID=$(id -g)
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth

USER_ROOT="$(pwd)"


docker run \
	-it \
	-p 8888:8888          \
	-w /home/camerauser          \
	--privileged=true \
	--net=host \
	--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
	--volume=/tmp/.docker.xauth:/tmp/.docker.xauth:rw \
	--volume=/dev/bus/usb:/dev/bus/usb:rw \
	--volume=/dev:/dev:rw \
	--env="XAUTHORITY=${XAUTH}" \
	--env="USER_UID=${USER_UID}" \
	--env="USER_GID=${USER_GID}" \
	--env="DISPLAY=${DISPLAY}" \
	--name=evo_test \
	evo/evo_main \

export containerId='docker ps -l -q'
