# Paparazzi installer: the components that can be installed.
#
# This file is the recipe for installing Paparazzi: the installer menus (install.sh), the Docker images
# (docker/*/Dockerfile) and the manual installation in the documentation all use it.
#
# A component <id> is added to COMPONENTS and defines:
#   <id>_title          one line shown in the menus
#   <id>_desc           longer description (GUI tooltip, --list)
#   <id>_default        1: selected by default when not installed yet
#   <id>_needs          components that are installed first
#   <id>_available      function, optional: return 1 (and print the reason) when it can't be installed here
#   <id>_packages       function, optional: apt packages, per OS version
#   <id>_python         function, optional: python modules, one per line: "<module> <apt package|-> <pip requirement|->"
#   <id>_install        function, optional: extra commands, after the packages (use run, see install_lib.sh)
#   <id>_check          function, optional: return 0 when installed (default: all packages and python modules)
#
# Helpers (install_lib.sh): ver_ge, ver_le, py_ver_ge, is_ubuntu, has_apt, run, apt_install, add_ppa, add_osrf_repo,
# submodule, udev_running, user_needs_group, user_name, has_cmd. OS_ID, OS_VER (e.g. 24.04), OS_CODENAME and PPRZ_DIR are set.
# Test the OS with these helpers, not on this computer directly: the documentation lists what each component
# installs per Ubuntu version from these functions (doc_packages.sh).

COMPONENTS=()

# ── Core ──────────────────────────────────────────────────────────────────────

COMPONENTS+=(core)
core_title="Paparazzi core (PPA, paparazzi-dev, build tools)"
core_desc="The Paparazzi apt repository and the packages needed to build Paparazzi and run the Paparazzi Center."
core_default=1
core_needs=""
core_available() { needs_ppa; }
core_packages() {
  echo paparazzi-dev python-is-python3 cmake libboost-program-options-dev libboost-filesystem-dev
  # plot and messages tools on 24.04 (26.04: lablgtk3, with paparazzi-dev)
  ver_ge 24.04 && ! ver_ge 26.04 && echo liblablgtk2-ocaml-dev
  # joystick tools: SDL 1.2 headers, not pulled in by paparazzi-dev on 26.04
  ver_ge 26.04 && echo libsdl1.2-dev
  return 0
}
core_repos() { add_ppa paparazzi-uav/ppa; }

COMPONENTS+=(python)
python_title="Python packages of the Paparazzi Center and tools"
python_desc="The Python modules of requirements.txt: from apt (system), in the pprzEnv venv or in the active conda environment."
python_default=1
python_needs="core"
python_python() {
  cat <<'EOF'
lxml python3-lxml lxml
numpy python3-numpy numpy
scipy python3-scipy scipy
ivy ivy-python ivy-python
serial python3-serial pyserial
PyQt5 python3-pyqt5 PyQt5
PyQt5.QtWebEngineWidgets python3-pyqt5.qtwebengine PyQtWebEngine
pyqtgraph python3-pyqtgraph pyqtgraph
OpenGL python3-opengl PyOpenGL
unidecode python3-unidecode unidecode
matplotlib python3-matplotlib matplotlib
distro python3-distro distro
pexpect python3-pexpect pexpect
pyubx2 - pyubx2
dronecan - dronecan
EOF
  # the DroneCAN code generator (sw/ext/dronecan) needs empy 3, Ubuntu 26.04 has empy 4
  if ver_ge 26.04; then echo "em - empy<=3.3.4"; else echo "em python3-empy empy<=3.3.4"; fi
  # the MAVLink code generator (sw/ext/mavlink, mavgen.py) needs future, Ubuntu 26.04 has no python3-future
  if ver_ge 26.04; then echo "future - future"; else echo "future python3-future future"; fi
}

# ── Compilers ─────────────────────────────────────────────────────────────────

COMPONENTS+=(arm)
arm_title="Autopilot compiler (gcc-arm-none-eabi, gdb)"
arm_desc="Cross-compiler and debugger for the STM32 autopilots (ChibiOS)."
arm_default=1
arm_needs=""
arm_available() { needs_apt; }
arm_packages() { echo gcc-arm-none-eabi libnewlib-arm-none-eabi gdb-multiarch; }

COMPONENTS+=(bebop)
bebop_title="Bebop and Disco compiler (arm-linux-gnueabi)"
bebop_desc="Cross-compiler for the Parrot Bebop 1/2 and Disco, and the Python module for the Bebop upload tools."
bebop_default=0
bebop_needs=""
# what a build may ask for (./install.sh --for=..., conf/Makefile.on_demand)
bebop_provides="arm-linux-gnueabi-gcc"
bebop_available() { needs_apt; }
bebop_packages() { echo gcc-arm-linux-gnueabi g++-arm-linux-gnueabi; }
bebop_python() {
  # the Bebop tools use telnetlib, removed from Python 3.13
  py_ver_ge 3.13 && echo "telnetlib3 - telnetlib3"
  return 0
}
bebop_install() {
  if ver_ge 24.04; then
    note "Ubuntu 24.04 and newer: the Bebop cameras need a 32-bit time_t, see https://github.com/paparazzi/paparazzi/issues/3762"
  fi
}

# ── Simulation ────────────────────────────────────────────────────────────────

COMPONENTS+=(parrot_2012)
parrot_2012_title="Parrot 2012.03 Bebop compiler (GCC 4.6.3, /opt/arm-2012.03)"
parrot_2012_desc="The original Parrot cross-compiler, for the airframes that ask for it with PREFIX=/opt/arm-2012.03/bin/arm-none-linux-gnueabi-. Downloaded from Parrot, checked (sha256) and unpacked in /opt/arm-2012.03, with the 32-bit C library to run it."
parrot_2012_default=0
parrot_2012_needs=""
parrot_2012_provides="/opt/arm-2012.03/bin/arm-none-linux-gnueabi-gcc"
PARROT_2012_URL=https://firmware.parrot.com/Toolchains/parrot-tools-linuxgnutools-2012.03_1.0.0-1_i386.deb
PARROT_2012_SHA256=84fa8c2782bd0f71ab39b905bf9833b1a8bc78419390c5a1ef6503ef74154c9c
parrot_2012_available() {
  needs_apt || return 1
  [ "$(dpkg --print-architecture 2>/dev/null)" = amd64 ] && return 0
  echo "only on amd64: the compiler is a 32-bit x86 program"
  return 1
}
# libc6-i386: the 32-bit C library (an amd64 package, no i386 architecture needed)
parrot_2012_packages() { echo libc6-i386 curl; }
parrot_2012_install() {
  local deb="$PPRZ_DIR/var/downloads/${PARROT_2012_URL##*/}"
  run mkdir -p "$PPRZ_DIR/var/downloads" || return 1
  if [ ! -f "$deb" ] || [ "${DRY_RUN:-0}" = 1 ]; then
    run curl -fL -o "$deb" "$PARROT_2012_URL" || return 1
  fi
  if ! run sh -c "echo '$PARROT_2012_SHA256  $deb' | sha256sum -c -"; then
    [ "${DRY_RUN:-0}" = 1 ] || rm -f "$deb"
    return 1
  fi
  # only the compiler: the package has no install scripts, and outside /opt only its changelog
  run $SUDO sh -c "dpkg-deb --fsys-tarfile '$deb' | tar -x -C / ./opt/arm-2012.03"
}
parrot_2012_check() { /opt/arm-2012.03/bin/arm-none-linux-gnueabi-gcc --version >/dev/null 2>&1; }

COMPONENTS+=(jsbsim)
jsbsim_title="JSBSim flight dynamics for the simulator (NPS)"
jsbsim_desc="The JSBSim library used by the NPS simulator of fixed wings and some rotorcraft."
jsbsim_default=1
jsbsim_needs="core"
jsbsim_available() { needs_ppa; }
jsbsim_packages() { echo paparazzi-jsbsim; }

COMPONENTS+=(gazebo)
gazebo_title="Gazebo simulator (gz Harmonic or Jetty)"
gazebo_desc="The new Gazebo (gz-sim) from packages.osrfoundation.org, with the TU Delft Gazebo models."
gazebo_default=0
gazebo_needs=""
gazebo_available() {
  is_ubuntu && ver_ge 22.04 && return 0
  echo "Ubuntu 22.04 or newer"; return 1
}
gazebo_provides="gz"
gazebo_packages() { if ver_ge 26.04; then echo gz-jetty; else echo gz-harmonic; fi; }
gazebo_repos() { add_osrf_repo; }
gazebo_install() { submodule sw/ext/tudelft_gazebo_models; }

COMPONENTS+=(gazebo_classic)
gazebo_classic_title="Gazebo Classic simulator (Gazebo 11)"
gazebo_classic_desc="Gazebo 11 (Ubuntu 20.04: from packages.osrfoundation.org, 22.04: from Ubuntu), with the TU Delft Gazebo models (MAV course)."
gazebo_classic_default=0
gazebo_classic_needs=""
gazebo_classic_available() {
  is_ubuntu && ver_le 22.04 && return 0
  echo "Ubuntu 22.04 or older (use Gazebo, or the docker/gazebo-classic image with the docker component)"; return 1
}
# packages.osrfoundation.org has gazebo11 for 20.04 only, Ubuntu 22.04 has gazebo 11.10
gazebo_classic_packages() { if ver_ge 22.04; then echo gazebo libgazebo-dev; else echo gazebo11 libgazebo11-dev; fi; }
gazebo_classic_repos() { ver_ge 22.04 || add_osrf_repo; }
gazebo_classic_install() { submodule sw/ext/tudelft_gazebo_models; }

COMPONENTS+=(docker)
docker_title="Docker (CI images, Gazebo Classic on newer Ubuntu)"
docker_desc="Docker from Ubuntu, to run the CI images (docker/) or the Gazebo Classic image (docker/gazebo-classic) on Ubuntu 24.04 and newer. Adds your user to the docker group."
docker_default=0
docker_needs=""
docker_available() { needs_apt; }
docker_packages() {
  # Docker installed in another way (docker-ce from docker.com): keep it
  has_cmd docker && return 0
  echo docker.io docker-buildx
}
docker_install() {
  if user_needs_group docker; then
    run $SUDO usermod -aG docker "$(user_name)"
    note "Log out and in again to use docker without sudo (group docker)"
  fi
}
docker_check() { command -v docker >/dev/null 2>&1 && ! user_needs_group docker; }

# ── Ground station and hardware ───────────────────────────────────────────────

COMPONENTS+=(gcs)
gcs_title="PprzGCS ground station"
gcs_desc="The PprzGCS ground control station (binary package from the Paparazzi PPA)."
gcs_default=1
gcs_needs="core"
gcs_available() { needs_ppa; }
gcs_packages() { echo pprzgcs; }

COMPONENTS+=(usb)
usb_title="USB access and flashing (udev rules, dfu-util)"
usb_desc="Udev rules for the autopilots and radios, dfu-util to flash, and the dialout/plugdev groups for your user."
usb_default=1
usb_needs=""
usb_available() { needs_apt; }
usb_packages() { echo dfu-util; }
usb_install() {
  run $SUDO mkdir -p /etc/udev/rules.d
  run $SUDO cp "$PPRZ_DIR"/conf/system/udev/rules/*.rules /etc/udev/rules.d/
  # reload only when udev runs (not in Docker)
  if udev_running; then
    run $SUDO udevadm control --reload-rules
  fi
  if user_needs_group dialout; then
    run $SUDO usermod -aG dialout,plugdev "$(user_name)"
    note "Log out and in again to use the serial ports (group dialout)"
  fi
}
usb_check() {
  local f
  for f in "$PPRZ_DIR"/conf/system/udev/rules/*.rules; do
    cmp -s "$f" "/etc/udev/rules.d/$(basename "$f")" || return 1
  done
  pkgs_installed $(usb_packages)
}

# ── Extras ────────────────────────────────────────────────────────────────────

COMPONENTS+=(opencv_bebop)
opencv_bebop_title="OpenCV for the Bebop (sw/ext/opencv_bebop)"
opencv_bebop_desc="The OpenCV submodule for computer vision on the Bebop, and the libraries to build it."
opencv_bebop_default=0
opencv_bebop_needs="bebop"
opencv_bebop_available() { needs_apt; }
opencv_bebop_packages() {
  echo cmake libjpeg-turbo8-dev libpng-dev libtiff-dev zlib1g-dev
  if ver_ge 22.04; then echo libdc1394-dev; else echo libdc1394-22-dev; fi
}
opencv_bebop_install() { submodule sw/ext/opencv_bebop; }
opencv_bebop_check() { pkgs_installed $(opencv_bebop_packages) && [ -e "$PPRZ_DIR/sw/ext/opencv_bebop/opencv/CMakeLists.txt" ]; }

COMPONENTS+=(video)
video_title="Video, joystick and motion capture tools"
video_desc="VLC, GStreamer and ffmpeg for the video streams, the joystick test tool, and the Python modules of the RTP viewer, NatNet (Optitrack) and moving base tools."
video_default=0
video_needs=""
video_available() { needs_apt; }
video_packages() { echo ffmpeg vlc gstreamer1.0-tools gstreamer1.0-plugins-good jstest-gtk; }
video_python() {
  cat <<'EOF'
cv2 python3-opencv opencv-python
pymap3d python3-pymap3d pymap3d
pyquaternion - pyquaternion
EOF
  return 0
}

COMPONENTS+=(docs)
docs_title="Documentation build tools (Sphinx, Doxygen)"
docs_desc="To build the user documentation (doc/sphinx) and the code documentation (make dox) locally."
docs_default=0
docs_needs=""
docs_available() { needs_apt; }
docs_packages() { echo doxygen graphviz python3-sphinx python3-sphinx-rtd-theme python3-myst-parser; }

# ── Build ─────────────────────────────────────────────────────────────────────

COMPONENTS+=(build)
build_title="Build Paparazzi (make)"
build_desc="Get the submodules and build Paparazzi, so the Paparazzi Center can start. Takes a while the first time."
build_default=1
build_needs="core python"
build_install() {
  # with the Python packages in the venv, build with it (as ./paparazzi does)
  if [ "$PY_MODE" = venv ]; then
    run env VIRTUAL_ENV="$PPRZ_DIR/$VENV_NAME" PATH="$PPRZ_DIR/$VENV_NAME/bin:$PATH" make -C "$PPRZ_DIR"
  else
    run make -C "$PPRZ_DIR"
  fi
}
build_check() { [ -x "$PPRZ_DIR/sw/ground_segment/tmtc/link" ] && [ -d "$PPRZ_DIR/var/include/pprzlink" ]; }
