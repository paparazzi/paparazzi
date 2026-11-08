#!/usr/bin/env bash
# Get a git submodule of paparazzi (e.g. sw/ext/tudelft_gazebo_models).
#
# usage: steps/submodule.sh <path>        (DRY_RUN=1 steps/submodule.sh ...: only show the commands)

set -e
. "$(dirname "$0")/common.sh"
check_args 1 "$@"

# the documentation: the command to type in the paparazzi folder
if [ "$DOC" = 1 ]; then
  run git submodule update --init "$1"
  exit 0
fi
if [ "$SIM" != 1 ] && [ ! -e "$PPRZ_DIR/.git" ]; then
  echo "  Note: $1: not a git clone, get the submodule by hand"
  exit 0
fi
run git -C "$PPRZ_DIR" submodule sync -- "$1"
run git -C "$PPRZ_DIR" submodule update --init -- "$1"
