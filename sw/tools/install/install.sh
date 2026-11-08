#!/usr/bin/env bash
# Paparazzi installer, started by ./install.sh in the paparazzi folder (./install.sh --help).
#
# The installer is install.py, run with the system Python (/usr/bin/python3, 3.8 or newer; never the Python of
# conda or of a venv). This script only makes sure that Python is there: in a bare Docker image it installs the
# apt package python3 first (about 30 MB). Everything else is in install.py.

INSTALL_DIR=$(cd "$(dirname "$0")" && pwd)
PYTHON=/usr/bin/python3
DOCS_URL=https://paparazzi-uav.readthedocs.io/en/latest/getting_started/install.html
HELP="Install by hand: $DOCS_URL, or use a Docker image: docker/README.md"

# macOS: /usr/bin/python3 is only real with the Xcode command line tools (otherwise it asks to install them)
if [ "$(uname -s)" = Darwin ] && ! xcode-select -p > /dev/null 2>&1; then
  echo "macOS: this installer installs with apt (Ubuntu or Debian). $HELP"
  exit 1
fi

# The minor version of the system Python 3 (e.g. 12 for 3.12), empty when there is none
python_minor() {
  "$PYTHON" --version 2>&1 | sed -n 's/^Python 3\.\([0-9]*\).*/\1/p'
}

# Run install.py when the system Python is new enough, or say what to do when it is too old
start_installer() {
  minor=$(python_minor)
  if [ -n "$minor" ] && [ "$minor" -ge 8 ]; then
    exec "$PYTHON" "$INSTALL_DIR/install.py" "$@"
  fi
  if [ -n "$minor" ]; then
    echo "Python 3.$minor is too old for this installer (3.8 or newer: Ubuntu 20.04 and newer). $HELP"
    exit 1
  fi
}

start_installer "$@"

# No python3 yet (e.g. a bare Docker image)
yes=0
dry_run=0
for arg in "$@"; do
  case "$arg" in
    -y|--yes) yes=1 ;;
    -n|--dry-run) dry_run=1 ;;
  esac
done
if ! command -v apt-get > /dev/null 2>&1; then
  echo "This installer needs python3 and apt (Ubuntu or Debian). $HELP"
  exit 1
fi
# "sudo " with a space, so that the commands below read well without sudo (root, Docker)
sudo=""
if [ "$(id -u)" -ne 0 ]; then
  sudo="sudo "
fi
echo "The installer needs Python 3 first:"
echo "  \$ ${sudo}apt-get update"
echo "  \$ ${sudo}env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends python3"
if [ "$dry_run" = 1 ]; then
  echo "(dry run: the rest needs python3)"
  exit 1
fi
if [ "$yes" = 0 ]; then
  if [ ! -t 0 ]; then
    echo "Run it in a terminal, or add --yes"
    exit 1
  fi
  printf 'Install it now? [Y/n] '
  read -r answer
  case "$answer" in
    n|N|no|No) exit 1 ;;
  esac
fi
$sudo apt-get update || exit 1
$sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends python3 || exit 1
start_installer "$@"
echo "No $PYTHON after installing python3. $HELP"
exit 1
