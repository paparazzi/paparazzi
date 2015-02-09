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

# options to grant access to the Xserver
X_WINDOW_OPTS="--volume=$XSOCK:$XSOCK --volume=$XAUTH:$XAUTH --env=XAUTHORITY=${XAUTH} --env=DISPLAY=${DISPLAY}"

# pass audio to pulseaudio server on host
PULSE_AUDIO_OPTS="--volume=/run/user/${USER_UID}/pulse:/run/pulse"

# give the container access to USB, WARNING runs it as priviliged container!
# use it if ENABLE_USB variable is non-empty/zero
if [ -n "$ENABLE_USB" ]; then
    echo "INFO: running as priviliged container to enable USB access!"
    USB_OPTS="--privileged --volume=/dev/bus/usb:/dev/bus/usb"
fi

# share the paparazzi directory and set it as working directory
SHARE_PAPARAZZI_HOME_OPTS="--volume=$PAPARAZZI_HOME:$PPRZ_HOME_CONTAINER \
  --env=PAPARAZZI_HOME=$PPRZ_HOME_CONTAINER \
  --env=PAPARAZZI_SRC=$PPRZ_HOME_CONTAINER \
  -w $PPRZ_HOME_CONTAINER"

docker run \
    ${X_WINDOW_OPTS} \
    ${PULSE_AUDIO_OPTS} \
    ${USB_OPTS} \
    ${SHARE_PAPARAZZI_HOME_OPTS} \
    --rm $args

# cleanup XAUTHORITY file again
rm -f $XAUTH
