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
SUDO=sudo
PY_MODE=system
set_colors 0
has_apt() { return 0; }
REPOS=(); CMDS=()
run() { CMDS+=("$*"); }
note() { :; }
add_ppa() { REPOS+=("ppa:$1"); }
add_osrf_repo() { REPOS+=("packages.osrfoundation.org (Gazebo)"); }
submodule() { CMDS+=("git submodule update --init $1"); }

# the description of component $1 on the current OS_VER, one item per line
describe() {
  local id=$1 reason lines
  if ! reason=$(comp_available "$id"); then
    echo "not available: ${reason:-not on this version}"
    return
  fi
  REPOS=(); CMDS=()
  declare -F "${id}_repos" >/dev/null && "${id}_repos"
  [ ${#REPOS[@]} -gt 0 ] && echo "repository: $(printf '``%s`` ' "${REPOS[@]}")"
  local pkgs; pkgs=$(comp_packages "$id" | xargs)
  [ -n "$pkgs" ] && echo "apt: $(printf '``%s`` ' $pkgs)"
  lines=$(comp_python "$id")
  if [ -n "$lines" ]; then
    local apt pip
    apt=$(awk 'NF && $2 != "-" {print $2}' <<< "$lines" | xargs)
    pip=$(awk 'NF && $2 == "-" {print $3}' <<< "$lines" | xargs)
    [ -n "$apt" ] && echo "Python, apt: $(printf '``%s`` ' $apt)"
    [ -n "$pip" ] && echo "Python, pip only: $(printf '``%s`` ' $pip)"
  fi
  [ "$id" = python ] && echo "venv or conda, the pprzlink library: \`\`pip install -e sw/ext/pprzlink/lib/v2.0/python\`\`"
  declare -F "${id}_install" >/dev/null && "${id}_install" >/dev/null
  local c
  for c in "${CMDS[@]}"; do echo "command: \`\`$c\`\`"; done
  return 0
}

for id in "${COMPONENTS[@]}"; do
  h="\`\`$id\`\`: $(comp_var "$id" title)"
  printf '%s\n%s\n\n' "$h" "$(printf '%*s' "${#h}" '' | tr ' ' '~')"
  printf '%s\n' "$(comp_var "$id" desc)"
  needs=$(comp_var "$id" needs)
  [ -n "$needs" ] && printf 'Needs: %s.\n' "$(printf '``%s`` ' $needs | sed 's/ $//')"
  echo
  # the Ubuntu versions with the same description together
  declare -A text=(); order=()
  for v in "$@"; do
    OS_ID=ubuntu OS_VER=$v OS_CODENAME=${CODENAME[$v]} PY_VER=${PYTHON[$v]}
    d=$(describe "$id")
    [ -n "${text[$d]+x}" ] || order+=("$d")
    text[$d]="${text[$d]:+${text[$d]}, }$v"
  done
  for d in "${order[@]}"; do
    printf -- '- **Ubuntu %s**\n\n' "${text[$d]}"
    [ -n "$d" ] || d="nothing to install"
    while IFS= read -r line; do printf '  - %s\n' "$line"; done <<< "$d"
    echo
  done
  unset text
done
