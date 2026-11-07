#!/usr/bin/env bash
#
# Paparazzi installer, started by install.sh in the paparazzi folder.
#
# With a desktop it shows a window (zenity), otherwise a menu in the terminal, to choose what to install.
# The components and what they install are in components.sh. Bash only: it runs before anything is installed.

INSTALL_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PPRZ_DIR=$(cd "$INSTALL_DIR/../../.." && pwd)
DOCS_URL="https://paparazzi-uav.readthedocs.io/en/latest/getting_started/install.html"

# shellcheck source=install_lib.sh
. "$INSTALL_DIR/install_lib.sh"
# shellcheck source=components.sh
. "$INSTALL_DIR/components.sh"

usage() {
  cat <<EOF
Usage: ./install.sh [options] [component ...]

Without components: choose them in a window (zenity) or a menu in the terminal.
With components: install those (and the ones they need), e.g. ./install.sh --yes core arm jsbsim

Options:
  -y, --yes             don't ask for confirmation (for scripts and Docker)
  -n, --dry-run         only show the commands
      --python=MODE     where the Python packages go: system (apt, default), venv ($VENV_NAME) or conda
      --text            menu in the terminal, also with a desktop
      --gui             window (zenity)
      --list            list the components and whether they are installed
      --for=FILE|COMMAND  install the component that provides it, after asking (used by the build:
                        conf/Makefile.on_demand). Exit 0: there, 1: not installed, 2: no component has it
      --no-recommends   apt without recommended packages (Docker images)
  -h, --help            this help

Components: ${COMPONENTS[*]}
            default: the components selected by default
Documentation: $DOCS_URL
EOF
}

# ── Arguments ─────────────────────────────────────────────────────────────────

ASSUME_YES=0; DRY_RUN=0; UI=auto; LIST=0; NO_RECOMMENDS=0; PY_MODE=""; ARGS_IDS=(); FOR=""
NOTES=()
for arg in "$@"; do
  case "$arg" in
    -y|--yes) ASSUME_YES=1 ;;
    -n|--dry-run) DRY_RUN=1 ;;
    --python=*) PY_MODE=${arg#--python=} ;;
    --for=*) FOR=${arg#--for=} ;;
    --text) UI=text ;;
    --gui) UI=gui ;;
    --list) LIST=1 ;;
    --no-recommends) NO_RECOMMENDS=1 ;;
    -h|--help) set_colors 0; usage; exit 0 ;;
    -*) echo "Unknown option $arg (see --help)" >&2; exit 2 ;;
    *) ARGS_IDS+=("$arg") ;;
  esac
done

if [ -t 1 ] && [ -z "${NO_COLOR:-}" ]; then set_colors 1; else set_colors 0; fi
detect_os

# Python mode: the given one, else the existing venv or the active conda environment, else system
if [ -z "$PY_MODE" ]; then
  if "$PPRZ_DIR/$VENV_NAME/bin/python3" -c "" 2>/dev/null; then PY_MODE=venv
  elif [ -n "${CONDA_PREFIX:-}" ] && [ "${CONDA_DEFAULT_ENV:-base}" != base ]; then PY_MODE=conda
  # Ubuntu 26.04: pip is not allowed in the system Python, and the build needs empy 3 from pip
  elif ver_ge 26.04; then PY_MODE=venv
  else PY_MODE=system
  fi
fi
case "$PY_MODE" in system|venv|conda) ;; *) echo "--python must be system, venv or conda" >&2; exit 2 ;; esac
PY_MODES=(system venv conda)
PY_MODE_DESC_system="apt packages for the system Python (default)"
PY_MODE_DESC_venv="venv $VENV_NAME with the system packages, plus pip for the rest"
PY_MODE_DESC_conda="pip in the active conda environment${CONDA_DEFAULT_ENV:+ ($CONDA_DEFAULT_ENV)}"

if [ "$(id -u)" -eq 0 ]; then SUDO=""; else SUDO=sudo; fi

# ── State of the components ───────────────────────────────────────────────────

declare -A AVAIL STATUS SEL
scan() {
  local id reason
  for id in "${COMPONENTS[@]}"; do
    reason=$(comp_available "$id") && AVAIL[$id]="" || AVAIL[$id]=${reason:-not available here}
    if comp_installed "$id" 2>/dev/null; then STATUS[$id]=installed; else STATUS[$id]=""; fi
  done
}

select_defaults() {
  local id
  for id in "${COMPONENTS[@]}"; do
    SEL[$id]=0
    [ -z "${AVAIL[$id]}" ] && [ "$(comp_var "$id" default)" = 1 ] && [ -z "${STATUS[$id]}" ] && SEL[$id]=1
  done
  return 0
}

selected_ids() { local id; for id in "${COMPONENTS[@]}"; do [ "${SEL[$id]:-0}" = 1 ] && echo "$id"; done; return 0; }

list_components() {
  local id state
  printf '%s%-16s %-12s %s%s\n' "$BOLD" "component" "state" "description" "$NC"
  for id in "${COMPONENTS[@]}"; do
    if [ -n "${AVAIL[$id]}" ]; then state="${GREY}n/a        "
    elif [ -n "${STATUS[$id]}" ]; then state="${GREEN}installed  "
    else state="${YELLOW}-          "; fi
    printf '%-16s %s%s %s\n' "$id" "$state" "$NC" "$(comp_var "$id" title)"
    [ -n "${AVAIL[$id]}" ] && printf '%-16s %s%s%s\n' "" "$GREY" "  (${AVAIL[$id]})" "$NC"
  done
  return 0
}

# ── Menu in the terminal ──────────────────────────────────────────────────────

tui_draw() {
  local cur=$1 i=0 id box mark line
  printf '\033[H\033[2J'
  printf '%s%s  Paparazzi installer%s — %s\n' "$CYAN" "$BOLD" "$NC" "$OS_NAME"
  printf '%s  ↑↓/jk move · space select · a all · n none · ←→/p Python packages · enter install · q quit%s\n\n' "$GREY" "$NC"
  for id in "${COMPONENTS[@]}"; do
    if [ -n "${AVAIL[$id]}" ]; then box="[-]"
    elif [ "${SEL[$id]}" = 1 ]; then box="[x]"
    else box="[ ]"; fi
    mark=""
    [ -n "${STATUS[$id]}" ] && mark="${GREEN}✔ installed${NC}"
    [ -n "${AVAIL[$id]}" ] && mark="${GREY}${AVAIL[$id]}${NC}"
    line=$(printf '%s %-52s' "$box" "$(comp_var "$id" title)")
    if [ "$i" = "$cur" ]; then
      printf '  %s> %s%s %s\n' "$REV" "$line" "$NC" "$mark"
    elif [ -n "${AVAIL[$id]}" ]; then
      printf '    %s%s%s %s\n' "$GREY" "$line" "$NC" "$mark"
    else
      printf '    %s %s\n' "$line" "$mark"
    fi
    i=$((i + 1))
  done
  printf '\n'
  line="Python packages:"
  local m
  for m in "${PY_MODES[@]}"; do
    if [ "$m" = "$PY_MODE" ]; then line+=" (•) $m"; else line+=" ( ) $m"; fi
  done
  if [ "$i" = "$cur" ]; then printf '  %s> %s%s\n' "$REV" "$line" "$NC"; else printf '    %s\n' "$line"; fi
  local v="PY_MODE_DESC_$PY_MODE"
  printf '    %s%s%s\n\n' "$GREY" "${!v}" "$NC"
  # description of the component under the cursor
  [ "$cur" -lt "${#COMPONENTS[@]}" ] && printf '  %s%s%s\n' "$GREY" "$(comp_var "${COMPONENTS[$cur]}" desc)" "$NC"
}

py_mode_next() {
  local step=$1 i
  for i in "${!PY_MODES[@]}"; do [ "${PY_MODES[$i]}" = "$PY_MODE" ] && break; done
  PY_MODE=${PY_MODES[$(( (i + step + 3) % 3 ))]}
}

tui_select() {
  local cur=0 n=${#COMPONENTS[@]} key rest id
  printf '\033[?25l'
  trap 'printf "\033[?25h"' EXIT
  while true; do
    tui_draw "$cur"
    IFS= read -rsn1 key
    if [ "$key" = $'\033' ]; then
      read -rsn2 -t 0.05 rest
      key="ESC$rest"
    fi
    id=${COMPONENTS[$cur]:-}
    case "$key" in
      "ESC[A"|k) cur=$(( (cur + n) % (n + 1) )) ;;
      "ESC[B"|j) cur=$(( (cur + 1) % (n + 1) )) ;;
      "ESC[C"|l|p) py_mode_next 1 ;;
      "ESC[D"|h) py_mode_next -1 ;;
      " ")
        if [ "$cur" = "$n" ]; then py_mode_next 1
        elif [ -z "${AVAIL[$id]}" ]; then SEL[$id]=$(( 1 - SEL[$id] )); fi ;;
      a) for id in "${COMPONENTS[@]}"; do [ -z "${AVAIL[$id]}" ] && SEL[$id]=1; done ;;
      n) for id in "${COMPONENTS[@]}"; do SEL[$id]=0; done ;;
      q|Q|ESC) printf '\033[?25h\n'; exit 0 ;;
      "") break ;;
    esac
  done
  printf '\033[?25h\033[H\033[2J'
}

tui_confirm() {
  local answer
  [ "$ASSUME_YES" = 1 ] && return 0
  local id
  printf '\n  Install:\n'
  for id in "$@"; do printf '    %s\n' "$(comp_var "$id" title)"; done
  printf '  Python packages: %s\n\n  Continue? [Y/n] ' "$PY_MODE"
  read -r answer
  case "$answer" in n|N|no|No) return 1 ;; esac
}

# ── Window (zenity) ───────────────────────────────────────────────────────────

zen() { zenity "$@" 2>/dev/null; }

gui_select() {
  local args=() id state text skipped=""
  for id in "${COMPONENTS[@]}"; do
    if [ -n "${AVAIL[$id]}" ]; then
      skipped+=$'\n'"  - $(comp_var "$id" title): ${AVAIL[$id]}"
      continue
    fi
    if [ "${SEL[$id]}" = 1 ]; then args+=(TRUE); else args+=(FALSE); fi
    state="";
    [ -n "${STATUS[$id]}" ] && state="installed"
    args+=("$id" "$(comp_var "$id" title)" "$state" "$(comp_var "$id" desc)")
  done
  text="Choose what to install on $OS_NAME. Installed parts can be selected again to update them."
  [ -n "$skipped" ] && text+=$'\n'"Not available here:$skipped"
  local out
  out=$(zen --list --checklist --title="Paparazzi installer" --width=1100 --height=620 --text="$text" \
    --column="" --column=id --column=Component --column=State --column=Description --hide-column=2 \
    --print-column=2 --separator=' ' "${args[@]}") || exit 0
  for id in "${COMPONENTS[@]}"; do SEL[$id]=0; done
  for id in $out; do SEL[$id]=1; done

  # Python packages
  local needs_py=0
  for id in $(selected_ids); do [ -n "$(comp_python "$id")" ] && needs_py=1; done
  if [ "$needs_py" = 1 ]; then
    args=()
    local m v
    for m in "${PY_MODES[@]}"; do
      v="PY_MODE_DESC_$m"
      if [ "$m" = "$PY_MODE" ]; then args+=(TRUE); else args+=(FALSE); fi
      args+=("$m" "${!v}")
    done
    out=$(zen --list --radiolist --title="Paparazzi installer" --width=700 --height=300 \
      --text="Where should the Python packages go?" --column="" --column=Mode --column=Description "${args[@]}") || exit 0
    [ -n "$out" ] && PY_MODE=$out
  fi
}

gui_confirm() {
  [ "$ASSUME_YES" = 1 ] && return 0
  local titles="" id
  for id in "$@"; do titles+=$'\n'"  - $(comp_var "$id" title)"; done
  zen --question --title="Paparazzi installer" --width=600 \
    --text="Install:$titles"$'\n\n'"Python packages: $PY_MODE. Every command is shown while it runs."
}

# sudo without a terminal: ask the password in a window
gui_sudo() {
  [ -n "$SUDO" ] || return 0
  local askpass; askpass=$(mktemp)
  printf '#!/bin/sh\nzenity --password --title="Paparazzi installer: password for sudo" 2>/dev/null\n' > "$askpass"
  chmod +x "$askpass"
  export SUDO_ASKPASS=$askpass
  SUDO="sudo -A"
}

# ── Installing ────────────────────────────────────────────────────────────────

install_all() {
  local id failed=() done_=()
  printf '%sPaparazzi installer%s — %s — %s\n' "$BOLD" "$NC" "$OS_NAME" "$(date '+%Y-%m-%d %H:%M')"
  printf 'Python packages: %s\n' "$PY_MODE"
  for id in "$@"; do
    if comp_install "$id"; then done_+=("$id"); else failed+=("$id"); fi
  done
  header "Summary"
  for id in "${done_[@]}"; do ok "$(comp_var "$id" title)"; done
  for id in "${failed[@]}"; do fail "$(comp_var "$id" title)"; done
  for id in "${NOTES[@]}"; do printf '  %sℹ %s%s\n' "$YELLOW" "$id" "$NC"; done
  if [ ${#failed[@]} -gt 0 ]; then
    printf '\n  %sCheck the errors above and the documentation: %s%s\n' "$RED" "$DOCS_URL" "$NC"
    return 1
  fi
  if [ -n "$FOR" ]; then
    printf '\n  %sDone.%s The build continues.\n' "$GREEN$BOLD" "$NC"
  else
    printf '\n  %sDone.%s Start the Paparazzi Center with: %s./paparazzi%s\n' "$GREEN$BOLD" "$NC" "$BOLD" "$NC"
  fi
}

# log without colors in var/install.log
log_to() { tee >(sed -u 's/\x1b\[[0-9;?]*[A-Za-z]//g' >> "$1"); }

# ── On demand: the build needs a file or command ──────────────────────────────
#
# ./install.sh --for=<file or command>, e.g. from conf/Makefile.on_demand while compiling an airframe.
# Finds the component whose <id>_provides has it, asks (terminal, or a window without terminal) and installs
# it with the components it needs. All output goes to stderr (make reads stdout).
# Exit: 0 there or installed, 1 not installed (refused, failed or can't ask), 2 no component provides it.

comp_provider() {
  local id p
  for id in "${COMPONENTS[@]}"; do
    for p in $(comp_var "$id" provides); do [ "$p" = "$1" ] && { echo "$id"; return 0; }; done
  done
  return 1
}

for_install() {
  local need=$1 id reason question todo=()
  # already there (a file, or a command in the PATH)
  { [ -e "$need" ] || command -v "$need" >/dev/null 2>&1; } && return 0
  id=$(comp_provider "$need") || return 2
  if comp_installed "$id" 2>/dev/null; then
    echo "$need: $(comp_var "$id" title) is installed, but $need is not there" >&2
    return 1
  fi
  if ! reason=$(comp_available "$id"); then
    echo "$need: $(comp_var "$id" title) is not available here: $reason" >&2
    return 1
  fi
  mapfile -t todo < <(comp_resolve "$id")
  question="This build needs $need, from the installer component '$id': $(comp_var "$id" title)."
  question+=$'\n'"$(comp_var "$id" desc)"
  if [ "$ASSUME_YES" = 1 ]; then
    :
  elif [ -t 0 ]; then
    # make started in a terminal. Not /dev/tty: the Paparazzi Center may have been started from a terminal,
    # its make has a pipe as stdin and the question would wait, unseen, in that terminal.
    local answer
    printf '\n%s\n%sInstall it now (%s) [y/N]? %s' "$question" "$BOLD" "${todo[*]}" "$NC" >&2
    read -r answer
    case "$answer" in y|Y|yes) ;; *) echo "Not installed. Later: ./install.sh $id" >&2; return 1 ;; esac
  elif { [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]; } && command -v zenity >/dev/null 2>&1; then
    zen --question --title="Paparazzi installer" --width=600 \
      --text="$question"$'\n\n'"Install it now (${todo[*]})?" || { echo "Not installed. Later: ./install.sh $id" >&2; return 1; }
    set_colors 0
    gui_sudo
    [ "$DRY_RUN" = 1 ] || [ -z "$SUDO" ] || $SUDO -v || return 1
    mkdir -p "$PPRZ_DIR/var"
    install_all "${todo[@]}" 2>&1 | log_to "$PPRZ_DIR/var/install.log" | \
      zen --text-info --auto-scroll --title="Paparazzi installer (log in var/install.log)" --width=1000 --height=650
    return "${PIPESTATUS[0]}"
  else
    echo "$question" >&2
    echo "Install it with: ./install.sh $id" >&2
    return 1
  fi
  if [ "$DRY_RUN" = 0 ] && [ -n "$SUDO" ]; then
    if [ -t 0 ]; then
      sudo -v || return 1
    else
      # no terminal to type the password in (sudo would ask in /dev/tty, maybe unseen): only without password
      SUDO="sudo -n"
      sudo -n -v 2>/dev/null || { echo "Installing needs sudo: run in a terminal: ./install.sh $id" >&2; return 1; }
    fi
  fi
  mkdir -p "$PPRZ_DIR/var"
  install_all "${todo[@]}" 2>&1 | log_to "$PPRZ_DIR/var/install.log" >&2
  return "${PIPESTATUS[0]}"
}

# ── Main ──────────────────────────────────────────────────────────────────────

if [ -n "$FOR" ]; then
  for_install "$FOR"
  exit $?
fi

scan
if [ "$LIST" = 1 ]; then
  list_components
  exit 0
fi

if [ "$UI" = auto ]; then
  if [ ${#ARGS_IDS[@]} -gt 0 ]; then UI=none
  elif { [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]; } && command -v zenity >/dev/null 2>&1; then UI=gui
  elif [ -t 0 ]; then UI=text
  else
    echo "No terminal and no desktop: give the components, e.g. ./install.sh --yes core arm (see --help)" >&2
    exit 2
  fi
fi
[ "$UI" = gui ] && ! command -v zenity >/dev/null 2>&1 && { echo "zenity is not installed, use --text" >&2; exit 2; }

select_defaults
if [ ${#ARGS_IDS[@]} -gt 0 ]; then
  for id in "${COMPONENTS[@]}"; do SEL[$id]=0; done
  for id in "${ARGS_IDS[@]}"; do
    if [ "$id" = default ]; then
      for d in "${COMPONENTS[@]}"; do [ "$(comp_var "$d" default)" = 1 ] && [ -z "${AVAIL[$d]}" ] && SEL[$d]=1; done
      continue
    fi
    if [ -z "$(comp_var "$id" title)" ]; then echo "Unknown component $id (see --list)" >&2; exit 2; fi
    if [ -n "${AVAIL[$id]}" ]; then echo "$id: ${AVAIL[$id]}" >&2; exit 2; fi
    SEL[$id]=1
  done
fi

case "$UI" in
  gui) gui_select ;;
  text) tui_select ;;
esac

mapfile -t TODO < <(comp_resolve $(selected_ids))
if [ ${#TODO[@]} -eq 0 ]; then
  echo "Nothing selected."
  exit 0
fi

if [ "$PY_MODE" = conda ] && [ -z "${CONDA_PREFIX:-}" ]; then
  for id in "${TODO[@]}"; do
    if [ -n "$(comp_python "$id")" ]; then
      echo "Python packages in conda: activate the conda environment first (conda activate <env>)" >&2
      [ "$UI" = gui ] && zen --error --text="Python packages in conda: activate the conda environment first (conda activate &lt;env&gt;)"
      exit 2
    fi
  done
fi

if [ "$UI" = gui ]; then gui_confirm "${TODO[@]}" || exit 0
elif [ -t 0 ]; then tui_confirm "${TODO[@]}" || exit 0
fi

mkdir -p "$PPRZ_DIR/var"
LOG="$PPRZ_DIR/var/install.log"
if [ "$UI" = gui ] && [ ! -t 0 ]; then
  # started from the desktop, no terminal: password and output in windows
  gui_sudo
  [ "$DRY_RUN" = 1 ] || [ -z "$SUDO" ] || $SUDO -v || exit 1
  set_colors 0
  install_all "${TODO[@]}" 2>&1 | log_to "$LOG" | \
    zen --text-info --auto-scroll --title="Paparazzi installer (log in var/install.log)" --width=1000 --height=650
  exit "${PIPESTATUS[0]}"
fi

# sudo password once, before the output goes through the log
if [ "$DRY_RUN" = 0 ] && [ -n "$SUDO" ]; then
  sudo -v || exit 1
fi
if [ "$DRY_RUN" = 1 ]; then
  install_all "${TODO[@]}"
  exit
fi
install_all "${TODO[@]}" 2>&1 | log_to "$LOG"
status=${PIPESTATUS[0]}
echo "  Log: var/install.log"
exit "$status"
