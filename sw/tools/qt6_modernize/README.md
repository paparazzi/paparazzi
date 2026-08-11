# qt6_modernize

Bring the native **Qt6/C++** ground-segment apps under `sw/` (logalizer, tmtc,
simulator) up to the common Qt6 application coding standard, automatically.

> This is for the Qt apps only. The firmware **C** code keeps its own 2-space
> astyle profile (`fix_code_style.sh`) and is intentionally left untouched.

## What it does

A three-stage pipeline driven by the CMake **compile database**
(`var/build_qt/compile_commands.json`):

| # | Tool | Job | Config |
|---|------|-----|--------|
| 1 | `clang-tidy` | modern idioms & quality fixes (`modernize-*`, `performance-*`, `readability-*`, `bugprone-*`) | `qt6.clang-tidy` |
| 2 | `clazy-standalone` + `clang-apply-replacements` | Qt-semantic fixes clang-tidy can't do (old-style `connect`, `QString` allocations, range-loop detach) | built-in check set |
| 3 | `clang-format` | final layout: braces, 4-space indent, `Type *ptr`, `(int) cast` spacing, 100-col, include grouping | `qt6.clang-format` |

Stages run in this order on purpose: semantic rewrites first, cosmetic
formatting last so the diff is clean.

### Style chosen
The official **Qt coding style** (WebKit-derived: 4-space indent, `ColumnLimit
100`, `PointerAlignment: Right`, opening brace on its own line for
functions/classes, attached braces for control flow). It matches how these apps
are already authored and is the most common standard for Qt6 code.

## Why the configs live here (and are not dotfiles)
`qt6.clang-format` / `qt6.clang-tidy` are passed **explicitly**
(`--style=file:` / `--config-file=`). They are deliberately *not* named
`.clang-format` / `.clang-tidy`, so no editor or other tool auto-discovers them
and applies Qt style to the rest of the (C / OCaml) tree.

## Prerequisites
```sh
# clang-tidy + clang-apply-replacements are already present (LLVM 21).
sudo apt install clang-format clazy
```
The Qt apps must have been built once so the compile database exists:
```sh
make plotter      # or: make gcs
```
Missing tools are detected at run time and that stage is skipped with a hint —
the script never fails just because clazy or clang-format is absent.

## Usage
```sh
# Convert one file:
sw/tools/qt6_modernize/qt6-modernize.sh sw/logalizer/logplotter.cpp

# Convert a whole directory (recurses .cpp/.cc/.cxx/.h/.hpp/...):
sw/tools/qt6_modernize/qt6-modernize.sh sw/logalizer

# Preview only — report changes, touch nothing:
sw/tools/qt6_modernize/qt6-modernize.sh --check sw/logalizer/logplotter.cpp
```

| Option | Meaning |
|--------|---------|
| `-p, --build-dir DIR` | compile-DB dir (auto-detected by default) |
| `--check` | dry run; modify nothing |
| `--no-tidy` / `--no-clazy` / `--no-format` | skip a stage |
| `--checks STR` / `--clazy-checks STR` | override the check lists |
| `--exclude REGEX` | skip matching paths (e.g. don't convert `my_strange_utils.h`) |

The tool **edits in place** — run it on a clean git tree and review with
`git diff`.

## Safety notes
- `--header-filter` is scoped to the target's own directory, so a TU's Qt and
  system headers are **never** rewritten.
- C-style casts like `(int) x` have **no reliable autofix**; clang-tidy/clazy
  will flag them but they should be converted to `static_cast<int>(x)` by hand.
