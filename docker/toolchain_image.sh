#!/bin/sh
#
# usage: toolchain_image.sh <PREFIX>
#
# Prints the docker image that provides the cross-compiler <PREFIX>gcc (see toolchains.txt): the first
# image of that PREFIX that exists on this computer, otherwise the first one, built once from its
# Dockerfile. Prints nothing when docker can't be used or no image has this compiler.

PREFIX=$1
DIR=$(cd "$(dirname "$0")" && pwd)

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
echo "Building the docker image $1 for the cross-compiler $PREFIX (only once)..." >&2
docker build -q -t "$1" "$DIR/../$2" >&2 && echo "$1"
