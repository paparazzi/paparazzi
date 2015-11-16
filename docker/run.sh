#!/bin/sh

#set -x

# if no arguments given, start with interactive terminal
if test $# -lt 1; then
    args="-t -i flixr/pprz-dev /sbin/my_init -- bash"
else
    # Use this script with derived images, and pass your 'docker run' args
    args="$@"
fi

# check if running on Linux or OSX
UNAME=$(uname -s)


############################################################
# share this paparazzi directory with the container
############################################################

# set PAPARAZZI_SRC to this tree
# on OSX: readlink doesn't have the -f or -m options, try using pwd
if [ $UNAME == "Linux" ]; then
    SCRIPT=$(readlink -f $0)
    SCRIPT_DIR=$(dirname $(readlink -f $0))
    PAPARAZZI_SRC=$(readlink -m $SCRIPT_DIR/..)
else
    PAPARAZZI_SRC=$(dirname $(pwd))
fi

# PAPARAZZI_HOME inside the container
PPRZ_HOME_CONTAINER=/home/pprz/paparazzi
# share the paparazzi directory and set it as working directory
SHARE_PAPARAZZI_HOME_OPTS="--volume=$PAPARAZZI_SRC:$PPRZ_HOME_CONTAINER \
  --env=PAPARAZZI_HOME=$PPRZ_HOME_CONTAINER \
  --env=PAPARAZZI_SRC=$PPRZ_HOME_CONTAINER \
  -w $PPRZ_HOME_CONTAINER"


############################################################
# grant access to X-Server
############################################################
if [ $UNAME == "Linux" ]; then
    XSOCK=/tmp/.X11-unix
    XAUTH=/tmp/.docker.xauth
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

    # options to grant access to the Xserver
    X_WINDOW_OPTS="--volume=$XSOCK:$XSOCK --volume=$XAUTH:$XAUTH --env=XAUTHORITY=${XAUTH} --env=DISPLAY=${DISPLAY}"
fi

# using xauth with docker on OSX doesn't work, so we use socat:
# see https://github.com/docker/docker/issues/8710
if [ $UNAME == "Darwin" ]; then
    X_WINDOW_OPTS="--env=DISPLAY=192.168.99.1:0"
    TCPPROXY="socat TCP-LISTEN:6000,reuseaddr,fork UNIX-CLIENT:\"$DISPLAY\""
fi


############################################################
# Audio
############################################################
if [ $UNAME == "Linux" ]; then
    # pass audio to pulseaudio server on host
    USER_UID=$(id -u)
    PULSE_AUDIO_OPTS="--volume=/run/user/${USER_UID}/pulse:/run/pulse"
fi


############################################################
# USB
############################################################
# give the container access to USB, WARNING runs it as priviliged container!
# use it if ENABLE_USB variable is non-empty/zero
if [ -n "$PRIVILEGED_USB" ]; then
    echo "WARNING: running as priviliged container to enable complete USB access!"
    echo "Better pass devices explicitly: ./run.sh -i -t --device=/dev/ttyUSB0 flixr/pprz-dev bash"
    USB_OPTS="--privileged --volume=/dev/bus/usb:/dev/bus/usb"
fi

# try to detect which USB devices to pass to the container automatically
# set DISABLE_USB=1 to turn it off
if [ -z "$DISABLE_USB" ]; then
    # find on OSX doesn't have the -printf option... so use exec echo instead
    USB_OPTS=$(find /dev -maxdepth 1 \( -name "ttyACM?" -or -name "ttyUSB?" \) -exec echo -n "--device={} " \;)
    if [ -n "$USB_OPTS" ]; then
        echo Passing auto-detected USB devices: $USB_OPTS
    fi
fi


############################################################
# Run it!
############################################################

if [ $UNAME == "Darwin" ]; then
    # start socat in background to forward the X socket via TCP
    $TCPPROXY &
fi

# run the docker container with all the fancy options
docker run \
    ${X_WINDOW_OPTS} \
    ${PULSE_AUDIO_OPTS} \
    ${USB_OPTS} \
    ${SHARE_PAPARAZZI_HOME_OPTS} \
    --rm $args


############################################################
# cleanup after exiting from docker container
############################################################

# cleanup XAUTHORITY file again
rm -f $XAUTH

# on OSX kill background socat process again
if [ $UNAME == "Darwin" ]; then
    pkill -f "$TCPPROXY"
fi
