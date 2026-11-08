#!/usr/bin/env bash
#
# Install what Paparazzi needs: ./install.sh (window or terminal menu), ./install.sh --help for the options.
# The installer is in sw/tools/install, the components it installs in sw/tools/install/components.txt.

exec bash "$(cd "$(dirname "$0")" && pwd)/sw/tools/install/install.sh" "$@"
