#!/usr/bin/env bash
# Install the pprzlink Python library (sw/ext/pprzlink) in the venv pprzEnv or the active conda environment.
# Nothing to do for the system Python: it uses the library that make builds in var/lib/python.
#
# usage: steps/pprzlink.sh        (PY_MODE=venv or conda; DRY_RUN=1: only show the commands)

set -e
. "$(dirname "$0")/common.sh"
check_args 0 "$@"

if [ "$PY_MODE" = system ]; then
  exit 0
fi
py=$(py_bin)
if [ "$DRY_RUN" = 1 ] || ! "$py" -c 'import pprzlink' 2> /dev/null; then
  "$(dirname "$0")/submodule.sh" sw/ext/pprzlink
  run "$py" -m pip install -e "$PPRZ_DIR/sw/ext/pprzlink/lib/v2.0/python"
fi
