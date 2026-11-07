#!/usr/bin/env bash
#
# Print, in reStructuredText, what each installer component installs per Ubuntu version, from components.sh.
# The Sphinx documentation runs it at every build (doc/sphinx/source/conf.py), so the page follows the installer.
#
# Usage: doc_packages.sh [os ...]    os: 22.04 24.04 26.04 (default) or 20.04

INSTALL_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=install_lib.sh
. "$INSTALL_DIR/install_lib.sh"
# shellcheck source=components.sh
. "$INSTALL_DIR/components.sh"

[ $# -gt 0 ] || set -- 22.04 24.04 26.04

# codename and Python version of each Ubuntu release
declare -A CODENAME=([20.04]=focal [22.04]=jammy [24.04]=noble [26.04]=resolute)
declare -A PYTHON=([20.04]=3.8 [22.04]=3.10 [24.04]=3.12 [26.04]=3.14)

# describe a normal Ubuntu computer, record the commands instead of running them
DOC_MODE=1
PPRZ_DIR='~/paparazzi'
VENV_DIR="$PPRZ_DIR/$VENV_NAME"
SUDO=sudo
PY_MODE=system
set_colors 0
has_apt() { return 0; }
REPOS=(); CMDS=()
run() { CMDS+=("$*"); }
note() { :; }
add_ppa() { REPOS+=("sudo add-apt-repository -y ppa:$1"); }
add_osrf_repo() {
  local key=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  REPOS+=("sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o $key")
  REPOS+=("echo \"deb [arch=\$(dpkg --print-architecture) signed-by=$key] http://packages.osrfoundation.org/gazebo/ubuntu-stable $OS_CODENAME main\" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list")
}
submodule() { CMDS+=("git -C $PPRZ_DIR submodule update --init $1"); }

# "<command> <words...>" on lines of at most 88 characters (width of the code blocks), continued with " \\"
wrap_cmd() {
  local cmd=$1 line w; shift
  line=$cmd
  for w in "$@"; do
    if [ $(( ${#line} + ${#w} + 3 )) -gt 88 ]; then
      printf '%s \\\n' "$line"
      line="    $w"
    else
      line="$line $w"
    fi
  done
  printf '%s\n' "$line"
}

# quote the pip requirements with a version (empy<=3.3.4) for the shell
pip_args() { local a; for a in "$@"; do case "$a" in *[\<\>=]*) printf "'%s' " "$a" ;; *) printf '%s ' "$a" ;; esac; done; }

# the commands to install component $1 by hand on the current OS_VER, one per line,
# or one line "not available: <reason>"
describe() {
  local id=$1 reason lines
  if ! reason=$(comp_available "$id"); then
    echo "not available: ${reason:-not on this version}"
    return
  fi
  REPOS=(); CMDS=()
  declare -F "${id}_repos" >/dev/null && "${id}_repos"
  if [ ${#REPOS[@]} -gt 0 ]; then
    printf '%s\n' "${REPOS[@]}"
    echo "sudo apt-get update"
  fi
  local pkgs; pkgs=$(comp_packages "$id" | xargs)
  lines=$(comp_python "$id")
  local apt="" pip=""
  if [ -n "$lines" ]; then
    apt=$(awk 'NF && $2 != "-" {print $2}' <<< "$lines" | xargs)
    pip=$(awk 'NF && $2 == "-" {print $3}' <<< "$lines" | xargs)
  fi
  pkgs=$(echo $pkgs $apt | xargs)
  [ -n "$pkgs" ] && wrap_cmd "sudo apt-get install -y" $pkgs
  if [ "$id" = python ]; then
    echo "# the Python modules without Ubuntu package, in the venv $VENV_NAME (with the Ubuntu packages)"
    echo "python3 -m venv --system-site-packages $VENV_DIR"
    [ -n "$pip" ] && echo "$VENV_DIR/bin/pip install $(pip_args $pip | sed 's/ $//')"
    echo "git -C $PPRZ_DIR submodule update --init sw/ext/pprzlink"
    echo "$VENV_DIR/bin/pip install -e $PPRZ_DIR/sw/ext/pprzlink/lib/v2.0/python"
  elif [ -n "$pip" ]; then
    echo "# without Ubuntu package: in the venv $VENV_NAME (python component)"
    echo "$VENV_DIR/bin/pip install $(pip_args $pip | sed 's/ $//')"
  fi
  [ "$id" = build ] && echo "source $VENV_DIR/bin/activate   # when you made the venv $VENV_NAME"
  declare -F "${id}_install" >/dev/null && "${id}_install" >/dev/null
  [ ${#CMDS[@]} -gt 0 ] && printf '%s\n' "${CMDS[@]}"
  return 0
}

for id in "${COMPONENTS[@]}"; do
  h="\`\`$id\`\`: $(comp_var "$id" title)"
  printf '%s\n%s\n\n' "$h" "$(printf '%*s' "${#h}" '' | tr ' ' '~')"
  printf '%s\n' "$(comp_var "$id" desc)"
  needs=$(comp_var "$id" needs)
  [ -n "$needs" ] && printf 'Needs: %s.\n' "$(printf '``%s`` ' $needs | sed 's/ $//')"
  echo
  # the Ubuntu versions with the same commands together
  declare -A text=(); order=()
  for v in "$@"; do
    OS_ID=ubuntu OS_VER=$v OS_CODENAME=${CODENAME[$v]} PY_VER=${PYTHON[$v]}
    d=$(describe "$id")
    [ -n "${text[$d]+x}" ] || order+=("$d")
    text[$d]="${text[$d]:+${text[$d]}, }$v"
  done
  for d in "${order[@]}"; do
    case "$d" in
      "not available: "*) printf -- '- **Ubuntu %s**: %s\n\n' "${text[$d]}" "$d" ;;
      "") printf -- '- **Ubuntu %s**: nothing to install\n\n' "${text[$d]}" ;;
      *)
        printf -- '- **Ubuntu %s**\n\n  .. code-block:: bash\n\n' "${text[$d]}"
        while IFS= read -r line; do printf '      %s\n' "$line"; done <<< "$d"
        echo
        ;;
    esac
  done
  unset text
done
