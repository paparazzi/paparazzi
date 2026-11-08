#!/usr/bin/env bash
# Copy the udev rules of the autopilots and radios (conf/system/udev/rules), so that a normal user can use
# them over USB, and reload them when udev runs (not in Docker).
#
# usage: steps/udev_rules.sh        (DRY_RUN=1 steps/udev_rules.sh: only show the commands)

set -e
. "$(dirname "$0")/common.sh"
check_args 0 "$@"

run $SUDO mkdir -p /etc/udev/rules.d
run $SUDO cp "$PPRZ_DIR"/conf/system/udev/rules/*.rules /etc/udev/rules.d/
if [ "$SIM" = 1 ] || [ -S /run/udev/control ]; then
  run $SUDO udevadm control --reload-rules
fi
