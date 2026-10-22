#!/usr/bin/env bash
# Herelink over its Wi-Fi hotspot: switch it on if needed (with the cable connected), then check the link.
exec "$(dirname "$0")/herelink_tether.sh" wifi "$@"
