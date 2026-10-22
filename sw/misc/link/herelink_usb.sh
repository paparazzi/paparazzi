#!/usr/bin/env bash
# Herelink over USB tethering: switch it on if needed, then check the link (see herelink_tether.sh).
exec "$(dirname "$0")/herelink_tether.sh" usb "$@"
