#!/bin/bash

# ── Color definitions ─────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

USE_VENV=true
BASHRC_SOURCE_VENV=false

# ── Helper functions ──────────────────────────────────────────────────────────

print_header() {
  echo ""
  echo -e "${CYAN}${BOLD}════════════════════════════════════════════════════════════════════════════════${NC}"
  echo -e "${CYAN}${BOLD}  $1${NC}"
  echo -e "${CYAN}${BOLD}════════════════════════════════════════════════════════════════════════════════${NC}"
  echo ""
}

print_success() {
  echo ""
  echo -e "${GREEN}${BOLD}  ✔ $1${NC}"
  echo ""
}

print_error() {
  echo ""
  echo -e "${RED}${BOLD}  ✘ $1${NC}"
  echo -e "${RED}    Please check the error above and consult the documentation:${NC}"
  echo -e "${RED}    https://paparazzi-uav.readthedocs.io${NC}"
  echo ""
}

run_step() {
  # Usage: run_step "description" command arg1 arg2 ...
  local description="$1"
  shift
  echo -e "  ${YELLOW}▸${NC} ${description}..."
  if "$@"; then
    echo -e "  ${GREEN}✔${NC} ${description} ${GREEN}done${NC}"
    return 0
  else
    print_error "${description} FAILED"
    return 1
  fi
}

# Stop on error
set -e

# ── Parse arguments ───────────────────────────────────────────────────────────

for arg in "$@"
do
  if [ "$arg" = "-h" ] || [ "$arg" = "--help" ]
  then
    echo "Usage: ./install.sh [-n|--no-venv] [-h|---help]"
    echo "  -n, --no-venv   Do not use python virtual environment"
    echo "  -s, --source    Add venv source in ~/.bashrc"
    echo "  -h, --help      Print this help"
    exit 0
  fi

  if [ "$arg" = "-n" ] || [ "$arg" = "--no-venv" ]
  then
    USE_VENV=false
    echo -e "${YELLOW}  ⚠ Virtual environment will NOT be installed.${NC}"
  fi

  if [ "$arg" = "-s" ] || [ "$arg" = "--source" ]
  then
    BASHRC_SOURCE_VENV=true
  fi

done


if [ $VIRTUAL_ENV ]
then
  echo -e "${YELLOW}  ⚠ Cannot create venv from itself! Run 'deactivate' first if you want to recreate the venv.${NC}"
  USE_VENV = false
  BASHRC_SOURCE_VENV=false
fi


# ── Install Qt dependencies ──────────────────────────────────────────────────

print_header "Step 1/3 — Installing Qt platform dependencies"

# Solves: Could not load the Qt platform plugin "xcb"
if run_step "Installing Qt libraries" sudo apt install -y libxcb-xinerama0 libxcb-cursor0 libxkbcommon-x11-0 libglu1-mesa; then
  print_success "Qt dependencies installed successfully"
else
  exit 1
fi


# ── Set up Python environment ────────────────────────────────────────────────

print_header "Step 2/3 — Setting up Python environment"

if [ "$USE_VENV" = true ]
then
  run_step "Installing python3 and python3-venv" sudo apt install -y python3 python3-venv
  run_step "Creating virtual environment" python3 setup.py
  source pprzEnv/bin/activate
  print_success "Virtual environment created and activated"

  if [ "$BASHRC_SOURCE_VENV" = true ]
  then
    echo "source $(pwd)/pprzEnv/bin/activate" >> ~/.bashrc
    print_success "Virtual environment source added to ~/.bashrc"
  fi

else
  run_step "Installing python3 and PyQt5 (system-wide)" sudo apt-get install -f -y python3 python3-pyqt5
  print_success "Python packages installed system-wide"
fi


# ── Launch the graphical installer ───────────────────────────────────────────

print_header "Step 3/3 — Launching Paparazzi installer GUI"

echo -e "  ${YELLOW}▸${NC} Starting graphical installer..."
echo -e "  ${CYAN}  Follow the numbered steps in the GUI to complete the installation.${NC}"
echo ""

python3 ./sw/tools/install.py


