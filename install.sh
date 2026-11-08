#!/usr/bin/env bash
#
# Install what Paparazzi needs: ./install.sh (window or terminal menu), ./install.sh --help for the options.
# The installer is in sw/tools/install, the components it installs in sw/tools/install/components.sh.

exec bash "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/sw/tools/install/pprz_install.sh" "$@"
