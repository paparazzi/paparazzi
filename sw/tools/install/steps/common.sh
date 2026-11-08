# Paparazzi installer: what the step scripts (steps/*.sh) share. Sourced by each of them.
#
# Sections: settings (defaults when a step runs by itself), output (quote, run, note, usage, check_args),
# the user and the Python (user_name, needs_group, py_bin).

# ── Settings ──────────────────────────────────────────────────────────────────
#
# install.py sets: PPRZ_INSTALL=1, PPRZ_DIR (the paparazzi folder), SUDO ("sudo", "sudo -A"
# or empty for root), DRY_RUN (1: only show the commands), PY_MODE (system, venv or conda),
# VENV_NAME, SIM (1: a simulated fresh Ubuntu, --os) and DOC (1: the commands for the documentation).
# A step started by hand gets these defaults (DRY_RUN and PY_MODE can be given, e.g. DRY_RUN=1 steps/make.sh):
if [ -z "$PPRZ_INSTALL" ]; then
  PPRZ_DIR=$(cd "$(dirname "$0")/../../../.." && pwd)
  VENV_NAME=pprzEnv
  DRY_RUN=${DRY_RUN:-0}
  SIM=0
  DOC=0
  if [ "$(id -u)" -eq 0 ]; then
    SUDO=""
  else
    SUDO=sudo
  fi
  if [ -z "$PY_MODE" ] && [ -x "$PPRZ_DIR/$VENV_NAME/bin/python3" ]; then
    PY_MODE=venv
  elif [ -z "$PY_MODE" ]; then
    PY_MODE=system
  fi
fi

# ── Output ────────────────────────────────────────────────────────────────────

# Print argument $1 as it must be typed in a shell: in quotes only when the shell would split or redirect it
# (e.g. 'empy<=3.3.4'). ~, $USER and * stay as they are, so that a copied command works.
quote() {
  case "$1" in
    *\'*) printf '"%s"' "$1" ;;
    *[\ \<\>\|\&\;\(\)]*) printf "'%s'" "$1" ;;
    *) printf '%s' "$1" ;;
  esac
}

# Show a command (to copy), then run it, except with DRY_RUN=1. Returns 1 and sets FAILED=1 when it fails.
run() {
  local line arg
  line=""
  for arg in "$@"; do
    line="$line $(quote "$arg")"
  done
  echo "  \$$line"
  if [ "$DRY_RUN" = 1 ]; then
    return 0
  fi
  if ! "$@"; then
    echo "  FAILED: $*"
    FAILED=1
    return 1
  fi
}

# Show a message for the user. install.sh repeats them at the end.
note() {
  echo "  Note: $*"
  NOTES="$NOTES  Note: $*
"
}

# Print the header of the script that is running (its comment lines up to the first empty line).
usage() {
  sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'
}

# Check the arguments of a step script: check_args <count> "$@" (<count>+: at least <count>).
# With -h or --help: show the usage and exit 0. With another number of arguments: show it and exit 2.
check_args() {
  local count
  count=$1
  shift
  if [ "$1" = -h ] || [ "$1" = --help ]; then
    usage
    exit 0
  fi
  case "$count" in
    *+) if [ $# -ge "${count%+}" ]; then return; fi ;;
    *) if [ $# -eq "$count" ]; then return; fi ;;
  esac
  usage >&2
  exit 2
}

# ── The user and the Python ───────────────────────────────────────────────────

# Print the user to add to groups ($USER in the documentation).
user_name() {
  if [ "$DOC" = 1 ]; then
    echo '$USER'
  else
    id -un
  fi
}

# Exit code 0 when the user is not in group $1 yet (root needs no group; --os: a new user).
needs_group() {
  if [ "$SIM" = 1 ]; then
    return 0
  fi
  [ "$(id -u)" -ne 0 ] && ! id -nG | grep -w "$1" > /dev/null
}

# Print the Python of the Python mode.
py_bin() {
  case "$PY_MODE" in
    venv) echo "$PPRZ_DIR/$VENV_NAME/bin/python3" ;;
    conda) echo "${CONDA_PREFIX:-/nonexistent}/bin/python" ;;
    *) echo /usr/bin/python3 ;;
  esac
}
