#!/bin/bash

# ── Color definitions ─────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

USE_VENV=false
BASHRC_SOURCE_VENV=false
INSTALL_ALL=false

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
    echo "Usage: ./install.sh [-a|--all] [-v|--venv] [-s|--source] [-h|--help]"
    echo "  -a, --all       Install everything without the window (the Everything button), then build"
    echo "  -v, --venv      Use python virtual environment: pprzEnv (not in an active conda environment:"
    echo "                  the Python packages are then installed in that conda environment)"
    echo "  -s, --source    Add venv source in ~/.bashrc"
    echo "  -h, --help      Print this help"
    exit 0
  fi

  if [ "$arg" = "-v" ] || [ "$arg" = "--venv" ]
  then
    USE_VENV=true
    echo -e "${GREEN}  ⚠ Virtual environment will be installed.${NC}"
  fi

  if [ "$arg" = "-s" ] || [ "$arg" = "--source" ]
  then
    BASHRC_SOURCE_VENV=true
  fi

  if [ "$arg" = "-a" ] || [ "$arg" = "--all" ]
  then
    INSTALL_ALL=true
  fi

done


if [ $VIRTUAL_ENV ]
then
  echo -e "${YELLOW}  ⚠ Cannot create venv from itself! Run 'deactivate' first if you want to recreate the venv.${NC}"
  USE_VENV=false
  BASHRC_SOURCE_VENV=false
fi

# conda: an active conda environment with its own python3 (the same test as install.py)
IN_CONDA=false
if [ -n "$CONDA_PREFIX" ] && [ "$(python3 -c 'import sys; print(sys.prefix)' 2>/dev/null)" = "$CONDA_PREFIX" ]
then
  IN_CONDA=true
  if [ "$USE_VENV" = true ]
  then
    echo -e "${YELLOW}  ⚠ Conda environment '$CONDA_DEFAULT_ENV' is active: the Python packages go there, no venv. Run 'conda deactivate' first if you want the venv.${NC}"
    USE_VENV=false
    BASHRC_SOURCE_VENV=false
  fi
fi

# ── Update packages list ──────────────────────────────────────────────────

print_header "Step 0/3 — Update packages list"

if run_step "APT Update" sudo apt update; then
  print_success "Packages list updated successfully"
else
  exit 1
fi


# ── Install Qt dependencies ──────────────────────────────────────────────────

print_header "Step 1/3 — Installing Qt platform dependencies"

# Solves: Could not load the Qt platform plugin "xcb" (the PyQt5 of pip needs these libraries; the Ubuntu
# desktop has icccm4 and keysyms1 up to 24.04, not on 26.04)
if run_step "Installing Qt libraries" sudo apt install -y libxcb-xinerama0 libxcb-cursor0 libxkbcommon-x11-0 libglu1-mesa \
     libxcb-icccm4 libxcb-keysyms1 libxcb-randr0 libxcb-shape0 libxcb-sync1 libxcb-xfixes0; then
  print_success "Qt dependencies installed successfully"
else
  exit 1
fi

# Libraries of the pip packages of the tools, present on an Ubuntu desktop but not on a minimal install:
#  - libasound2 (ALSA sound) and libxfixes3 (X11): the Qt WebEngine of the Paparazzi Center (documentation tab)
#  - libusb-1.0-0 (USB): Open3D of the RotWing 3D visualizer
# libasound2 is called libasound2t64 from Ubuntu 24.04 on
if apt-cache show libasound2t64 >/dev/null 2>&1; then LIBASOUND=libasound2t64; else LIBASOUND=libasound2; fi
if run_step "Installing libraries of the Center documentation (sound, X11) and the 3D visualizer (USB)" \
     sudo apt install -y $LIBASOUND libxfixes3 libusb-1.0-0; then
  print_success "Libraries of the Python tools installed successfully"
else
  exit 1
fi


# ── Set up Python environment ────────────────────────────────────────────────

print_header "Step 2/3 — Setting up Python environment"

# conda: python3 is conda's, so the Python packages go into the active conda environment
# (a separate one keeps them apart from the rest: conda create -n paparazzi python && conda activate paparazzi)
if [ "$IN_CONDA" = true ]
then
  run_step "Installing the Python packages in conda environment '$CONDA_DEFAULT_ENV'" python3 -m pip install -r requirements.txt
  print_success "Python packages installed in conda environment '$CONDA_DEFAULT_ENV' (activate it before ./paparazzi)"

elif [ "$USE_VENV" = true ]
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
  run_step "Installing python3 and PyQt5 (system-wide)" sudo apt-get install -f -y python3 python3-pyqt5 python3-distro
  print_success "Python packages installed system-wide"
fi


# ── Install everything without the window ────────────────────────────────────

if [ "$INSTALL_ALL" = true ]
then
  print_header "Step 3/3 — Installing everything"
  exec python3 ./sw/tools/install.py --all
fi

# ── Launch the graphical installer ───────────────────────────────────────────

print_header "Step 3/3 — Launching Paparazzi installer GUI"

echo -e "  ${YELLOW}▸${NC} Starting graphical installer..."
echo -e "  ${CYAN}  Follow the numbered steps in the GUI to complete the installation.${NC}"
echo ""

python3 ./sw/tools/install.py


