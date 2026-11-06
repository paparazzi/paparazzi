#!/bin/bash
#
# usage: toolchain_run.sh <image> <command> [arguments]
#
# Runs one compiler command (gcc, g++, objcopy, ...) in a docker image of toolchains.txt, as the current
# user, with the paparazzi tree mounted at the same path, so all paths stay the same as on the computer.

IMAGE=$1
shift

SRC=${PAPARAZZI_SRC:-$(cd "$(dirname "$0")/.." && pwd)}
MOUNTS=(-v "$SRC:$SRC")
if [ -n "$PAPARAZZI_HOME" ] && [ "$PAPARAZZI_HOME" != "$SRC" ]; then
  MOUNTS+=(-v "$PAPARAZZI_HOME:$PAPARAZZI_HOME")
fi
case "$PWD/" in
  "$SRC"/*|"${PAPARAZZI_HOME:-$SRC}"/*) ;;
  *) MOUNTS+=(-v "$PWD:$PWD") ;;
esac

# no entrypoint: some images (e.g. gazebo-classic) print messages from theirs
exec docker run --rm --entrypoint "" -u "$(id -u):$(id -g)" -e HOME=/tmp "${MOUNTS[@]}" -w "$PWD" "$IMAGE" "$@"
