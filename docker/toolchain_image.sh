#!/bin/bash
#
# usage: toolchain_image.sh <PREFIX>
#
# Prints the docker image that provides the cross-compiler <PREFIX>gcc (see toolchains.txt): the first
# image of that PREFIX that exists on this computer, otherwise the first one, built once from its
# Dockerfile. Prints nothing when docker can't be used or no image has this compiler.
#
# Messages go to stderr and start with "[docker]" (shown in cyan by the Paparazzi Center).

PREFIX=$1
DIR=$(cd "$(dirname "$0")" && pwd)

# "[docker]" in cyan in a terminal; the Paparazzi Center colours it itself
if [ -t 2 ]; then TAG=$'\e[36m[docker]\e[0m'; else TAG="[docker]"; fi
msg() { echo "$TAG $*" >&2; }

command -v docker >/dev/null 2>&1 || exit 0
docker info >/dev/null 2>&1 || exit 0

FIRST=""
while read -r prefix image dockerfile_dir rest; do
  case "$prefix" in ''|'#'*) continue ;; esac
  [ "$prefix" = "$PREFIX" ] || continue
  if docker image inspect "$image" >/dev/null 2>&1; then
    echo "$image"
    exit 0
  fi
  [ -n "$FIRST" ] || FIRST="$image $dockerfile_dir"
done < "$DIR/toolchains.txt"

[ -n "$FIRST" ] || exit 0
set -- $FIRST
msg "The cross-compiler ${PREFIX}gcc is not installed: building the docker image $1 that has it,"
msg "from $2. This happens only once: it downloads the compilers (an image of about 600 MB) and takes a few minutes."
msg "(Install the compiler instead, or set USE_DOCKER_TOOLCHAIN=0, to skip this.)"
docker build -t "$1" "$DIR/../$2" 2>&1 | while IFS= read -r line; do [ -n "$line" ] && echo "$TAG $line" >&2; done
if [ "${PIPESTATUS[0]}" -ne 0 ]; then
  msg "Building the docker image $1 failed (see above)."
  exit 0
fi
msg "Docker image $1 ready."
echo "$1"
