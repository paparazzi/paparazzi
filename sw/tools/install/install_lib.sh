# Paparazzi installer: helpers for the components (components.sh) and the menus (pprz_install.sh).
# Bash only, no other dependency, so it runs before anything is installed.

# ── Output ────────────────────────────────────────────────────────────────────

set_colors() {
  if [ "$1" = 1 ]; then
    RED=$'\033[0;31m'; GREEN=$'\033[0;32m'; YELLOW=$'\033[1;33m'; CYAN=$'\033[0;36m'
    GREY=$'\033[0;90m'; BOLD=$'\033[1m'; REV=$'\033[7m'; NC=$'\033[0m'
  else
    RED=""; GREEN=""; YELLOW=""; CYAN=""; GREY=""; BOLD=""; REV=""; NC=""
  fi
}

header() { printf '\n%s%s══ %s %s%s\n\n' "$CYAN" "$BOLD" "$1" "$(printf '═%.0s' $(seq 1 $((70 - ${#1}))))" "$NC"; }
ok()     { printf '  %s✔%s %s\n' "$GREEN" "$NC" "$1"; }
fail()   { printf '  %s✘ %s%s\n' "$RED$BOLD" "$1" "$NC"; }
note()   { printf '  %sℹ %s%s\n' "$YELLOW" "$1" "$NC"; NOTES+=("$1"); }

# a command line to copy and paste: the arguments with shell characters (empy<=3.3.4, sh -c '...') quoted
shell_line() {
  local a out=()
  for a in "$@"; do
    if [[ "$a" =~ ^[][A-Za-z0-9_./:,@%+=~^-]+$ ]]; then out+=("$a")
    elif [[ "$a" != *"'"* ]]; then out+=("'$a'")
    elif [[ "$a" != *[\"\$\`\\]* ]]; then out+=("\"$a\"")
    else out+=("$(printf '%q' "$a")")
    fi
  done
  printf '%s' "${out[*]}"
}

# Run one command, show it first. DRY_RUN=1 only shows it.
run() {
  printf '  %s$ %s%s\n' "$CYAN" "$(shell_line "$@")" "$NC"
  [ "${DRY_RUN:-0}" = 1 ] && return 0
  if "$@"; then
    return 0
  fi
  fail "failed: $*"
  COMP_FAILED=1
  return 1
}

# ── OS ────────────────────────────────────────────────────────────────────────

detect_os() {
  OS_ID=unknown; OS_VER=0; OS_CODENAME=""; OS_NAME="unknown OS"
  if [ -r /etc/os-release ]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    OS_ID=${ID:-unknown}; OS_VER=${VERSION_ID:-0}; OS_CODENAME=${VERSION_CODENAME:-}
    OS_NAME=${PRETTY_NAME:-$OS_ID $OS_VER}
    # Ubuntu derivatives (Linux Mint, Pop!_OS, ...): use the Ubuntu release they are based on
    if [ "$OS_ID" != ubuntu ] && [ -n "${UBUNTU_CODENAME:-}" ]; then
      OS_CODENAME=$UBUNTU_CODENAME
      case "$UBUNTU_CODENAME" in
        focal) OS_VER=20.04 ;; jammy) OS_VER=22.04 ;; noble) OS_VER=24.04 ;; resolute) OS_VER=26.04 ;;
      esac
      OS_ID=ubuntu
    fi
  fi
}

# version as a number: 24.04 -> 2404
ver_num() { local v=${1:-0}; local major=${v%%.*} minor=0; [ "$v" != "$major" ] && minor=${v#*.}; minor=${minor%%.*}; echo $((10#${major:-0} * 100 + 10#${minor:-0})); }
ver_ge() { [ "$(ver_num "$OS_VER")" -ge "$(ver_num "$1")" ]; }
ver_le() { [ "$(ver_num "$OS_VER")" -le "$(ver_num "$1")" ]; }
is_ubuntu() { [ "$OS_ID" = ubuntu ]; }
has_apt() { command -v apt-get >/dev/null 2>&1; }

# Python version of the system: py_ver_ge 3.13 (PY_VER can be set, e.g. for the documentation)
py_ver_ge() {
  [ -n "${PY_VER:-}" ] || PY_VER=$(/usr/bin/python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null)
  [ "$(ver_num "${PY_VER:-0}")" -ge "$(ver_num "$1")" ]
}

# DOC_MODE=1 (doc_packages.sh): describe what is installed on a normal computer, not on this one
udev_running() { [ "${DOC_MODE:-0}" = 1 ] || [ -S /run/udev/control ]; }
# a program installed in another way (e.g. docker-ce): don't install the Ubuntu package over it
has_cmd() { [ "${DOC_MODE:-0}" != 1 ] && command -v "$1" >/dev/null 2>&1; }
user_name() { if [ "${DOC_MODE:-0}" = 1 ]; then echo '$USER'; else id -un; fi; }
user_needs_group() { [ "${DOC_MODE:-0}" = 1 ] || { [ "$(id -u)" -ne 0 ] && ! id -nG | grep -qw "$1"; }; }

# for <id>_available
needs_apt() { has_apt && return 0; echo "needs apt (Ubuntu or Debian), see the manual installation"; return 1; }
needs_ppa() { is_ubuntu && return 0; echo "needs the Paparazzi PPA (Ubuntu), see the manual installation"; return 1; }

# ── apt ───────────────────────────────────────────────────────────────────────

APT_NEED_UPDATE=1

apt_update() {
  [ "$APT_NEED_UPDATE" = 1 ] || return 0
  run $SUDO apt-get update && APT_NEED_UPDATE=0
}

apt_install() {
  [ $# -gt 0 ] || return 0
  apt_update
  local opts=(-y)
  [ "${NO_RECOMMENDS:-0}" = 1 ] && opts+=(--no-install-recommends)
  run $SUDO env DEBIAN_FRONTEND=noninteractive apt-get install "${opts[@]}" "$@"
}

pkgs_installed() {
  local p
  for p in "$@"; do
    dpkg-query -W -f='${Status}' "$p" 2>/dev/null | grep -q 'ok installed' || return 1
  done
}

apt_has() { [ -n "$(apt-cache policy "$1" 2>/dev/null | sed -n 's/^ *Candidate: //p' | grep -v '(none)')" ]; }

apt_sources_have() { grep -rqs "$1" /etc/apt/sources.list /etc/apt/sources.list.d/; }

add_ppa() {
  apt_sources_have "$1" && return 0
  if ! command -v add-apt-repository >/dev/null 2>&1; then
    apt_install software-properties-common
  fi
  run $SUDO add-apt-repository -y "ppa:$1"
  APT_NEED_UPDATE=1
}

# packages.osrfoundation.org (Gazebo)
add_osrf_repo() {
  apt_sources_have packages.osrfoundation.org/gazebo && return 0
  command -v curl >/dev/null 2>&1 || apt_install curl ca-certificates
  local key=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  run $SUDO curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o "$key"
  run $SUDO sh -c "echo 'deb [arch=$(dpkg --print-architecture) signed-by=$key] http://packages.osrfoundation.org/gazebo/ubuntu-stable $OS_CODENAME main' > /etc/apt/sources.list.d/gazebo-stable.list"
  APT_NEED_UPDATE=1
}

# ── git ───────────────────────────────────────────────────────────────────────

submodule() {
  if [ ! -e "$PPRZ_DIR/.git" ]; then
    note "$1: not a git clone, get the submodule by hand"
    return 0
  fi
  run git -C "$PPRZ_DIR" submodule sync -- "$1"
  run git -C "$PPRZ_DIR" submodule update --init -- "$1"
}

# ── Python ────────────────────────────────────────────────────────────────────
#
# PY_MODE:
#   system  apt python3-* packages for the system Python (default). Modules without apt package are installed with
#           pip --user where the system allows it (before Ubuntu 23.04), otherwise reported.
#   venv    the same apt packages, plus the venv pprzEnv (with the system packages) for the modules without apt package.
#           ./paparazzi uses pprzEnv when it exists.
#   conda   pip in the active conda environment.

VENV_NAME=pprzEnv

py_bin() {
  case "$PY_MODE" in
    venv) echo "$PPRZ_DIR/$VENV_NAME/bin/python3" ;;
    conda) echo "${CONDA_PREFIX:-/nonexistent}/bin/python" ;;
    *) echo /usr/bin/python3 ;;
  esac
}

# print the modules that python $1 can't import
py_missing() {
  local py=$1; shift
  [ $# -gt 0 ] || return 0
  if [ ! -x "$py" ]; then
    printf '%s\n' "$@"
    return 0
  fi
  "$py" - "$@" <<'EOF' 2>/dev/null
import importlib.util, sys
for m in sys.argv[1:]:
    try:
        ok = importlib.util.find_spec(m) is not None
    except Exception:
        ok = False
    if not ok:
        print(m)
EOF
}

py_externally_managed() {
  /usr/bin/python3 -c 'import os, sysconfig; exit(0 if os.path.exists(os.path.join(sysconfig.get_path("stdlib"), "EXTERNALLY-MANAGED")) else 1)' 2>/dev/null
}

# lines "<module> <apt|-> <pip|->" of a component
comp_python() { declare -F "${1}_python" >/dev/null && "${1}_python"; return 0; }

comp_py_modules() { comp_python "$1" | awk 'NF {print $1}'; }

py_setup_venv() {
  local venv="$PPRZ_DIR/$VENV_NAME"
  [ -x "$venv/bin/python3" ] && return 0
  /usr/bin/python3 -c 'import ensurepip' 2>/dev/null || apt_install python3-venv
  run /usr/bin/python3 -m venv --system-site-packages "$venv"
  [ -e "$venv/bin/calibrate.py" ] || run ln -s ../../sw/tools/calibration/calibrate.py "$venv/bin/calibrate.py"
}

# install the python modules of a component for PY_MODE
py_install() {
  local lines; lines=$(comp_python "$1")
  [ -n "$lines" ] || return 0
  local mod apt pip apts=() pips=() missing
  if [ "$PY_MODE" != conda ]; then
    while read -r mod apt pip; do
      [ -n "$mod" ] && [ "$apt" != - ] && { [ "${DRY_RUN:-0}" = 1 ] || apt_has "$apt"; } && apts+=("$apt")
    done <<< "$lines"
    apt_install "${apts[@]}"
  fi
  [ "$PY_MODE" = venv ] && py_setup_venv
  local py; py=$(py_bin)
  if [ "$PY_MODE" = conda ] && [ ! -x "$py" ]; then
    fail "no conda environment active: run 'conda activate <env>' first"
    COMP_FAILED=1
    return 1
  fi
  if [ "${DRY_RUN:-0}" = 1 ] && [ "$PY_MODE" != conda ]; then
    # nothing installed yet: the modules without apt package that the Python can't import now
    # (the venv may not exist yet: check with the system Python, but show the commands with the venv)
    local pycheck=$py
    [ -x "$pycheck" ] || pycheck=/usr/bin/python3
    missing=" $(py_missing "$pycheck" $(awk 'NF && $2 == "-" {print $1}' <<< "$lines") | tr '\n' ' ')"
  else
    missing=" $(py_missing "$py" $(awk 'NF {print $1}' <<< "$lines") | tr '\n' ' ')"
  fi
  while read -r mod apt pip; do
    [ -n "$mod" ] && [ "$pip" != - ] && [[ "$missing" == *" $mod "* ]] && pips+=("$pip")
  done <<< "$lines"
  [ ${#pips[@]} -gt 0 ] || return 0
  if [ "$PY_MODE" = system ]; then
    if py_externally_managed; then
      note "Not installed (no apt package, and pip is not allowed in the system Python): ${pips[*]}. Choose the venv (--python=venv) if you need them"
      return 0
    fi
    /usr/bin/python3 -m pip --version >/dev/null 2>&1 || apt_install python3-pip
    # as root (Docker images): for all users
    if [ "$(id -u)" -eq 0 ]; then
      run /usr/bin/python3 -m pip install "${pips[@]}"
    else
      run /usr/bin/python3 -m pip install --user "${pips[@]}"
    fi
  else
    run "$py" -m pip install "${pips[@]}"
  fi
}

# pprzlink python library in the venv/conda environment, as before (the tools also find the one built in var/lib/python)
py_install_pprzlink() {
  [ "$PY_MODE" = system ] && return 0
  local py; py=$(py_bin)
  if [ "${DRY_RUN:-0}" = 1 ] || ! "$py" -c 'import pprzlink' 2>/dev/null; then
    submodule sw/ext/pprzlink
    run "$py" -m pip install -e "$PPRZ_DIR/sw/ext/pprzlink/lib/v2.0/python"
  fi
}

# ── Components ────────────────────────────────────────────────────────────────

comp_var() { local v="${1}_$2"; echo "${!v:-}"; }

# print the reason and return 1 when the component can't be installed here
comp_available() { declare -F "${1}_available" >/dev/null || return 0; "${1}_available"; }

comp_packages() { declare -F "${1}_packages" >/dev/null && "${1}_packages"; return 0; }

comp_installed() {
  if declare -F "${1}_check" >/dev/null; then
    "${1}_check"
    return
  fi
  local pkgs; pkgs=$(comp_packages "$1")
  # shellcheck disable=SC2086
  [ -z "$pkgs" ] || pkgs_installed $pkgs || return 1
  [ -z "$(py_missing "$(py_bin)" $(comp_py_modules "$1"))" ]
}

comp_install() {
  local id=$1
  COMP_FAILED=0
  header "$(comp_var "$id" title)"
  declare -F "${id}_repos" >/dev/null && "${id}_repos"
  # shellcheck disable=SC2046
  apt_install $(comp_packages "$id")
  py_install "$id"
  [ "$id" = python ] && py_install_pprzlink
  declare -F "${id}_install" >/dev/null && "${id}_install"
  if [ "$COMP_FAILED" = 0 ]; then
    ok "$(comp_var "$id" title)"
  else
    fail "$(comp_var "$id" title)"
  fi
  return "$COMP_FAILED"
}

# selected components plus the ones they need, in the order of COMPONENTS
comp_resolve() {
  local -A want=()
  local id dep changed=1
  for id in "$@"; do want[$id]=1; done
  while [ "$changed" = 1 ]; do
    changed=0
    for id in "${!want[@]}"; do
      for dep in $(comp_var "$id" needs); do
        if [ -z "${want[$dep]:-}" ] && ! comp_installed "$dep"; then want[$dep]=1; changed=1; fi
      done
    done
  done
  for id in "${COMPONENTS[@]}"; do [ -n "${want[$id]:-}" ] && echo "$id"; done
  return 0
}
