#!/usr/bin/env bash
# Download a .deb into var/downloads, check its sha256 and unpack it in /. For packages without install
# scripts, e.g. the Parrot 2012.03 compiler (/opt/arm-2012.03). A file already downloaded is used again.
#
# usage: steps/unpack_deb.sh <url> <sha256>        (DRY_RUN=1 steps/unpack_deb.sh ...: only show the commands)

set -e
. "$(dirname "$0")/common.sh"
check_args 2 "$@"
url=$1
sha256=$2
deb="$PPRZ_DIR/var/downloads/$(basename "$url")"

run mkdir -p "$PPRZ_DIR/var/downloads"
if [ "$DRY_RUN" = 1 ] || [ ! -f "$deb" ]; then
  run curl -fL -o "$deb" "$url"
fi
# shown so that the documentation lists it: it must print <sha256>
run sha256sum "$deb"
if [ "$DRY_RUN" != 1 ] && [ "$(sha256sum "$deb" | cut -d ' ' -f 1)" != "$sha256" ]; then
  echo "  FAILED: wrong sha256 of $deb, expected $sha256: removed, try again"
  rm -f "$deb"
  exit 1
fi
run $SUDO dpkg-deb -x "$deb" /
