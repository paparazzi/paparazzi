#!/usr/bin/env bash
# Build Paparazzi (make in the paparazzi folder). With the Python modules in the venv pprzEnv, build with it
# (as ./paparazzi does).
#
# usage: steps/make.sh        (DRY_RUN=1 steps/make.sh: only show the command)

set -e
. "$(dirname "$0")/common.sh"
check_args 0 "$@"

if [ "$PY_MODE" = venv ]; then
  run env VIRTUAL_ENV="$PPRZ_DIR/$VENV_NAME" PATH="$PPRZ_DIR/$VENV_NAME/bin:$PATH" make -C "$PPRZ_DIR"
else
  run make -C "$PPRZ_DIR"
fi
