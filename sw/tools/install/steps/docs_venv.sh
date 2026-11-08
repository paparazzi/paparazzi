#!/usr/bin/env bash
# Make the venv of the documentation tools, doc/sphinx/venv, with the versions of doc/sphinx/requirements.txt
# (like Read the Docs). make doc uses it when it is there.
#
# usage: steps/docs_venv.sh        (DRY_RUN=1 steps/docs_venv.sh: only show the commands)

set -e
. "$(dirname "$0")/common.sh"
check_args 0 "$@"

# the documentation: the commands to type in the paparazzi folder
dir="$PPRZ_DIR/doc/sphinx"
if [ "$DOC" = 1 ]; then
  dir=doc/sphinx
fi
run /usr/bin/python3 -m venv "$dir/venv"
run "$dir/venv/bin/pip" install -r "$dir/requirements.txt"
