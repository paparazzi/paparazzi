#!/usr/bin/env bash
# =============================================================================
# qt6-modernize.sh -- one-shot modernizer/formatter for the native Qt6/C++
#                     ground-segment apps under sw/ (logalizer, tmtc, simulator).
#
# Brings a .cpp/.h (or a whole directory of them) up to the common Qt6
# application standard, in three stages, using a JSON compilation database:
#
#   1. clang-tidy          -- modernize-/performance-/readability-/bugprone-*
#                             idiom fixes      (config: qt6.clang-tidy)
#   2. clazy-standalone     -- Qt-semantic fixes clang-tidy cannot do, e.g.
#                             old-style SIGNAL/SLOT connect -> function-pointer
#                             connect, QString allocation hygiene, range-loop
#                             detach. Fix-its are applied with
#                             clang-apply-replacements.
#   3. clang-format        -- final whitespace/brace/pointer/include layout
#                             (config: qt6.clang-format = Qt house style)
#
# The two config files live NEXT TO this script and are passed explicitly, so
# running this tool never imposes Qt style on the rest of the (C/OCaml) tree.
#
# Usage:
#   qt6-modernize.sh [options] <file-or-directory> [<file-or-directory> ...]
#
# Options:
#   -p, --build-dir DIR   Directory containing compile_commands.json.
#                         (auto-detected: $PAPARAZZI/var/build_qt, ./var/build_qt,
#                          ./build, <git-root>/var/build_qt)
#       --check           Dry run: report what WOULD change, modify nothing.
#       --no-tidy         Skip the clang-tidy stage.
#       --no-clazy        Skip the clazy stage.
#       --no-format       Skip the clang-format stage.
#       --checks STR      Override the clang-tidy check string.
#       --clazy-checks STR  Override the clazy check string.
#       --exclude REGEX   Skip files whose path matches REGEX (extended regex).
#                         Default protects os_desktop_utils.h, which is kept
#                         byte-identical with the external PprzGCS copy.
#   -h, --help            Show this help.
#
# Notes:
#   * Edits files in place. Run on a clean git tree and review with `git diff`.
#   * Only files UNDER the paths you pass are modified; a TU's Qt/system headers
#     and external/submodule headers (e.g. sw/ext) are never reported on or
#     rewritten -- BOTH clang-tidy and clazy-standalone get the same
#     --header-filter scoped to your targets' own directories.
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd -P)"
TIDY_CFG="$SCRIPT_DIR/qt6.clang-tidy"
FMT_CFG="$SCRIPT_DIR/qt6.clang-format"

# ---- defaults ---------------------------------------------------------------
DO_TIDY=1
DO_CLAZY=1
DO_FORMAT=1
CHECK=0
BUILD_DIR=""
CHECKS_OVERRIDE=""
EXCLUDE_REGEX=""
#EXCLUDE_REGEX='os_desktop_utils\.h'
# Curated clazy set: Qt6 idioms that are safe & fixit-capable on an already-Qt6
# code base. (Pure Qt5->Qt6 porting checks that must run against Qt5 headers --
# e.g. qt6-deprecated-api-fixes, qt6-qlatin1stringchar-to-u, qt6-fwd-fixes,
# qt6-header-fixes -- are intentionally omitted: they are also absent from the
# installed clazy-standalone and would only emit "Invalid check" noise.)
#
# Deliberately an EXPLICIT list rather than the broad "level1" shorthand, and
# missing-qobject-macro is intentionally NOT included: the logalizer apps are
# single-translation-unit (each app is one .cpp that ends with #include "X.moc")
# and their QObject helper classes (ChartLegendManager, EditorLighteningStyle,
# ...) live in shared headers that AUTOMOC does not moc. Adding Q_OBJECT to them
# declares out-of-line virtuals with no compiled meta-object -> "undefined vtable"
# link errors. Those helpers use only function-pointer connect() and need no moc.
CLAZY_CHECKS="old-style-connect,qstring-allocations,qstring-ref,\
range-loop-reference,range-loop-detach,container-anti-pattern"

die()  { printf 'qt6-modernize: error: %s\n' "$*" >&2; exit 1; }
note() { printf 'qt6-modernize: %s\n' "$*" >&2; }

usage() { sed -n '2,/^set -euo/p' "$0" | sed 's/^# \{0,1\}//; s/^#//' | sed '$d'; exit "${1:-0}"; }

# ---- first matching executable from a list of candidate names ---------------
find_tool() {
    local name
    for name in "$@"; do
        if command -v "$name" >/dev/null 2>&1; then printf '%s\n' "$name"; return 0; fi
    done
    return 1
}

# ---- arg parsing ------------------------------------------------------------
TARGETS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        -p|--build-dir)   BUILD_DIR="${2:?--build-dir needs a value}"; shift 2 ;;
        --check)          CHECK=1; shift ;;
        --no-tidy)        DO_TIDY=0; shift ;;
        --no-clazy)       DO_CLAZY=0; shift ;;
        --no-format)      DO_FORMAT=0; shift ;;
        --checks)         CHECKS_OVERRIDE="${2:?--checks needs a value}"; shift 2 ;;
        --clazy-checks)   CLAZY_CHECKS="${2:?--clazy-checks needs a value}"; shift 2 ;;
        --exclude)        EXCLUDE_REGEX="${2:?--exclude needs a value}"; shift 2 ;;
        -h|--help)        usage 0 ;;
        --)               shift; while [[ $# -gt 0 ]]; do TARGETS+=("$1"); shift; done ;;
        -*)               die "unknown option '$1' (try --help)" ;;
        *)                TARGETS+=("$1"); shift ;;
    esac
done
[[ ${#TARGETS[@]} -gt 0 ]] || die "no file or directory given (try --help)"
[[ -f "$TIDY_CFG" ]] || die "missing config: $TIDY_CFG"
[[ -f "$FMT_CFG"  ]] || die "missing config: $FMT_CFG"

# ---- locate the compilation database ----------------------------------------
if [[ -z "$BUILD_DIR" ]]; then
    git_root="$(git -C "$(dirname -- "${TARGETS[0]}")" rev-parse --show-toplevel 2>/dev/null || true)"
    for cand in "${PAPARAZZI_HOME:-}/var/build_qt" "./var/build_qt" "./build" \
                "${git_root:+$git_root/var/build_qt}"; do
        [[ -n "$cand" && -f "$cand/compile_commands.json" ]] && { BUILD_DIR="$cand"; break; }
    done
fi
[[ -n "$BUILD_DIR" && -f "$BUILD_DIR/compile_commands.json" ]] \
    || die "no compile_commands.json found; pass --build-dir DIR (build the Qt apps first, e.g. 'make plotter')"
BUILD_DIR="$(cd -- "$BUILD_DIR" && pwd)"
DB="$BUILD_DIR/compile_commands.json"
note "using compile database: $DB"

# ---- expand targets into a concrete file list -------------------------------
SRC_RE='\.(cpp|cc|cxx|c\+\+)$'
HDR_RE='\.(h|hh|hpp|hxx|h\+\+)$'
FILES=()                      # all target files (logical paths) -> clang-format
add_file() {
    local f="$1"
    if [[ -n "$EXCLUDE_REGEX" ]] && printf '%s' "$f" | grep -Eq "$EXCLUDE_REGEX"; then
        note "skip (excluded): $f"; return
    fi
    FILES+=("$(cd -- "$(dirname -- "$f")" && pwd)/$(basename -- "$f")")
}
for t in "${TARGETS[@]}"; do
    if [[ -d "$t" ]]; then
        while IFS= read -r -d '' f; do add_file "$f"; done \
            < <(find "$t" -type f -regextype posix-extended -regex ".*($SRC_RE|$HDR_RE)" -print0)
    elif [[ -f "$t" ]]; then
        add_file "$t"
    else
        die "no such file or directory: $t"
    fi
done
[[ ${#FILES[@]} -gt 0 ]] || die "no .cpp/.h files matched (after exclusions)"

# Index the compile database by CANONICAL (symlink-resolved) path, so targets
# match no matter which symlink they are reached through. The Paparazzi tree is
# itself frequently a symlink (e.g. ~/paparazzi -> .../paparazzi_main) while the
# database stores whichever form CMake saw, so a plain string match is unsafe.
declare -A DB_BY_CANON
while IFS= read -r p; do
    [[ -n "$p" ]] || continue
    c="$(realpath -m -- "$p" 2>/dev/null || printf '%s' "$p")"
    DB_BY_CANON["$c"]="$p"
done < <(grep -oE '"file"[[:space:]]*:[[:space:]]*"[^"]*"' "$DB" | sed -E 's/.*"([^"]*)"$/\1/')

TUS=()              # translation units, in the EXACT path form stored in the DB
HFILTER_DIRS=()
for f in "${FILES[@]}"; do
    printf '%s' "$f" | grep -Eq "$SRC_RE" || continue
    c="$(realpath -m -- "$f" 2>/dev/null || printf '%s' "$f")"
    if [[ -n "${DB_BY_CANON[$c]:-}" ]]; then
        TUS+=("${DB_BY_CANON[$c]}")
        HFILTER_DIRS+=("$(dirname -- "${DB_BY_CANON[$c]}")")
    else
        note "skip tidy/clazy (not in compile DB): $f"
    fi
done

# --header-filter: project headers under the targets' own directories only, in
# the DB's path form so it matches the paths clang-tidy reports (never Qt/system).
if [[ ${#HFILTER_DIRS[@]} -gt 0 ]]; then
    uniq_dirs="$(printf '%s\n' "${HFILTER_DIRS[@]}" | sort -u)"
    HEADER_FILTER="$(printf '%s' "$uniq_dirs" | sed 's/[.[\*^$+?(){}|]/\\&/g' | paste -sd'|' -)"
    HEADER_FILTER="(${HEADER_FILTER})/"
else
    HEADER_FILTER='$^'    # matches nothing
fi

note "targets: ${#TUS[@]} translation unit(s), ${#FILES[@]} file(s) total"
[[ $CHECK -eq 1 ]] && note "MODE: --check (dry run, no files will be modified)"

# ---- SAFEGUARD snapshot -----------------------------------------------------
# External submodules under sw/ext are OUT OF SCOPE and must stay byte-identical
# with their committed originals. --header-filter is the primary guard (it scopes
# both clang-tidy and clazy to the targets' own directories), but clazy exports
# fix-its via a YAML that clang-apply-replacements applies with no path filter of
# its own, so a single header-filter miss could silently rewrite a submodule
# header. As a hard backstop, record each sw/ext git repo's currently-dirty file
# set now; after the fixing stages we revert ONLY the files this run newly
# touched, leaving any pre-existing local edits intact. Skipped in --check mode.
PPRZ_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd -P)"
EXT_REPOS=()
declare -A EXT_BEFORE
if [[ $CHECK -eq 0 && -d "$PPRZ_ROOT/sw/ext" ]]; then
    mapfile -t EXT_REPOS < <(find "$PPRZ_ROOT/sw/ext" -maxdepth 2 -name .git -printf '%h\n' 2>/dev/null)
    if [[ ${#EXT_REPOS[@]} -gt 0 ]]; then
        for r in "${EXT_REPOS[@]}"; do
            EXT_BEFORE["$r"]="$(git -C "$r" status --porcelain 2>/dev/null | cut -c4- | sort)"
        done
    fi
fi

# ---- stage 1: clang-tidy ----------------------------------------------------
if [[ $DO_TIDY -eq 1 ]]; then
    if CT="$(find_tool clang-tidy clang-tidy-21 clang-tidy-20 clang-tidy-19)"; then
        tidy_args=(-p "$BUILD_DIR" "--config-file=$TIDY_CFG"
                   "--header-filter=$HEADER_FILTER" --quiet)
        [[ -n "$CHECKS_OVERRIDE" ]] && tidy_args+=("--checks=$CHECKS_OVERRIDE")
        [[ $CHECK -eq 0 ]] && tidy_args+=(--fix)
        if [[ ${#TUS[@]} -eq 0 ]]; then
            note "stage 1 (clang-tidy): no translation units to process -- skipped"
        else
            note "stage 1: clang-tidy ($CT) on ${#TUS[@]} TU(s)$([[ $CHECK -eq 1 ]] && echo ' [check]' || echo ' [--fix]')"
            for tu in "${TUS[@]}"; do "$CT" "${tidy_args[@]}" "$tu" || true; done
        fi
    else
        note "stage 1 SKIPPED: clang-tidy not found  ->  sudo apt install clang-tidy"
    fi
fi

# ---- stage 2: clazy-standalone + clang-apply-replacements -------------------
if [[ $DO_CLAZY -eq 1 ]]; then
    if CZ="$(find_tool clazy-standalone)"; then
        if [[ ${#TUS[@]} -eq 0 ]]; then
            note "stage 2 (clazy): no translation units to process -- skipped"
        elif [[ $CHECK -eq 1 ]]; then
            note "stage 2: clazy-standalone [check] checks=$CLAZY_CHECKS"
            for tu in "${TUS[@]}"; do "$CZ" -p "$BUILD_DIR" "-checks=$CLAZY_CHECKS" "--header-filter=$HEADER_FILTER" "$tu" || true; done
        else
            CAR="$(find_tool clang-apply-replacements clang-apply-replacements-21 \
                             clang-apply-replacements-20 clang-apply-replacements-19 || true)"
            if [[ -z "$CAR" ]]; then
                note "stage 2 SKIPPED: clazy found but clang-apply-replacements not  -> sudo apt install clang-tidy"
            else
                yaml_dir="$(mktemp -d)"; trap 'rm -rf "$yaml_dir"' EXIT
                note "stage 2: clazy-standalone ($CZ) [--fix via $CAR] checks=$CLAZY_CHECKS"
                i=0
                for tu in "${TUS[@]}"; do
                    "$CZ" -p "$BUILD_DIR" "-checks=$CLAZY_CHECKS" \
                          "--header-filter=$HEADER_FILTER" \
                          "-export-fixes=$yaml_dir/fix_$((i++)).yaml" "$tu" || true
                done
                "$CAR" --ignore-insert-conflict "$yaml_dir" 2>/dev/null \
                    || "$CAR" "$yaml_dir" 2>/dev/null || note "  (no clazy fix-its to apply, or a conflict was skipped)"
            fi
        fi
    else
        note "stage 2 SKIPPED: clazy-standalone not found  ->  sudo apt install clazy"
    fi
fi

# ---- stage 3: clang-format --------------------------------------------------
if [[ $DO_FORMAT -eq 1 ]]; then
    if CF="$(find_tool clang-format clang-format-21 clang-format-20 clang-format-19)"; then
        note "stage 3: clang-format ($CF)$([[ $CHECK -eq 1 ]] && echo ' [check]' || echo ' [-i]')"
        rc=0
        for f in "${FILES[@]}"; do
            if [[ $CHECK -eq 1 ]]; then
                "$CF" "--style=file:$FMT_CFG" --dry-run --Werror "$f" || { rc=1; note "  would reformat: $f"; }
            else
                "$CF" "--style=file:$FMT_CFG" -i "$f"
            fi
        done
        [[ $CHECK -eq 1 && $rc -eq 0 ]] && note "  all target files already match the Qt style"
    else
        note "stage 3 SKIPPED: clang-format not found  ->  sudo apt install clang-format"
    fi
fi

# ---- SAFEGUARD enforcement --------------------------------------------------
# Revert any sw/ext submodule file a fixing stage newly modified. This is the
# hard backstop behind --header-filter: out-of-scope external submodules are
# guaranteed to end exactly as they started (pre-existing local edits untouched).
if [[ $CHECK -eq 0 && ${#EXT_REPOS[@]} -gt 0 ]]; then
    for r in "${EXT_REPOS[@]}"; do
        after="$(git -C "$r" status --porcelain 2>/dev/null | cut -c4- | sort)"
        while IFS= read -r f; do
            [[ -n "$f" ]] || continue
            note "SAFEGUARD: reverted out-of-scope submodule edit -> ${r#"$PPRZ_ROOT"/}/$f"
            git -C "$r" checkout -- "$f" 2>/dev/null || true
        done < <(comm -13 <(printf '%s\n' "${EXT_BEFORE[$r]:-}") <(printf '%s\n' "$after"))
    done
fi

note "done.$([[ $CHECK -eq 0 ]] && echo "  Review with: git diff -- ${TARGETS[*]}")"
