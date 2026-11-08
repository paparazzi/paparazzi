#!/usr/bin/env bash
# Add the user to groups (e.g. dialout,plugdev for the serial ports) when not in the first one yet.
# The new groups work after logging out and in again.
#
# usage: steps/group.sh <group,group,...>        (DRY_RUN=1 steps/group.sh ...: only show the commands)

set -e
. "$(dirname "$0")/common.sh"
check_args 1 "$@"

first_group=${1%%,*}
if needs_group "$first_group"; then
  run $SUDO usermod -aG "$1" "$(user_name)"
fi
