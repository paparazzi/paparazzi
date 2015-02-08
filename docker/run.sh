#!/bin/sh

#set -x

# if no arguments given, start with interactive terminal
if test $# -lt 1; then
	args="-t -i flixr/pprz-dev /sbin/my_init -- bash"
else
	# Use this script with derived images, and pass your 'docker run' args
	args="$@"
fi

if [ -z "$PAPARAZZI_HOME" ]; then
    SCRIPT=$(readlink -f $0)
    SCRIPT_DIR=$(dirname $(readlink -f $0))
    PAPARAZZI_HOME=$(readlink -m $SCRIPT_DIR/..)
fi

# PAPARAZZI_HOME inside the container
PPRZ_HOME_CONTAINER=/home/pprz/paparazzi


USER_UID=$(id -u)
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

docker run \
	--rm \
	--volume=/run/user/${USER_UID}/pulse:/run/pulse \
	--volume=$XSOCK:$XSOCK \
	--volume=$XAUTH:$XAUTH \
	--env="XAUTHORITY=${XAUTH}" \
	--env="DISPLAY=${DISPLAY}" \
    --volume=$PAPARAZZI_HOME:$PPRZ_HOME_CONTAINER \
    --env="PAPARAZZI_HOME=$PPRZ_HOME_CONTAINER" \
    --env="PAPARAZZI_SRC=$PPRZ_HOME_CONTAINER" \
	-u pprz \
    -w $PPRZ_HOME_CONTAINER \
	$args

# cleanup XAUTHORITY file again
rm -f $XAUTH
