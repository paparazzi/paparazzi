cmake_minimum_required(VERSION 3.16)
project(PaparazziQtSuperBuild LANGUAGES C CXX)

# ---------------------------------------------------------------------------
# Aggregating ("super") build for the native Qt6/C++ ground-segment & tool
# applications shipped with Paparazzi:
#
#     ground_segment/tmtc         -> messages
#     logalizer                   -> plotter, logplotter, play
#     simulator                   -> gaia
#
# (The GCS "cockpit" application is NOT part of this super-build: it is built
# separately from sw/ground_segment/cockpit via the top-level `make cockpit`,
# which reuses the same shared sw/ext Qt libraries.)
#
# This file lives at the root of the source tree it governs (sw/), which is the
# idiomatic CMake layout: every add_subdirectory() below is a plain, in-tree
# relative path -- no "../" reaching out of the listfile's own directory
#
# All applications are configured into a SINGLE build tree so the shared
# libraries (pprzlinkQt and IvyQt) are configured and compiled exactly once and
# then linked by every consumer. The canonical entry points are the per-app
# Makefiles, which delegate here through conf/Makefile.qt; you can also drive it
# directly, e.g.:
#
#     cmake -B var/build_qt -S sw -G Ninja -DCMAKE_BUILD_TYPE=Release
#     cmake --build var/build_qt --target messages gaia plotter logplotter play
# ---------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -s")

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    add_compile_options($<$<CONFIG:Release>:-march=native>)
endif()

# ---------------------------------------------------------------------------
# Faster linking: prefer mold, then lld, otherwise fall back to the default
# system linker. check_linker_flag() actually try-links a test program with the
# flag, so a linker is selected only when it is installed AND accepted by the
# active compiler -- this whole block is a safe no-op when neither is present.
# Set at directory scope so every target defined here, and every
# add_subdirectory() child added below (tmtc, simulator), inherits it.
# ---------------------------------------------------------------------------
if(NOT CMAKE_VERSION VERSION_LESS 3.18)
    include(CheckLinkerFlag)
    foreach(_pprz_linker mold lld)
        check_linker_flag(CXX "-fuse-ld=${_pprz_linker}" PPRZ_HAVE_LD_${_pprz_linker})
        if(PPRZ_HAVE_LD_${_pprz_linker})
            add_link_options(-fuse-ld=${_pprz_linker})
            message(STATUS "Faster linker enabled: -fuse-ld=${_pprz_linker}")
            break()
        endif()
    endforeach()
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# ---------------------------------------------------------------------------
# Qt6 is required to build every native ground-segment / tool app defined
# below. On hosts without a Qt6 development install (e.g. a firmware-only build
# server) we deliberately do NOT hard-fail the whole configure step: detect Qt6
# quietly and, when it is absent, emit a clear notice and skip the rest of this
# super-build instead of aborting with a fatal find_package() error. Install
# the Qt6 development packages and re-run CMake to actually build the apps.
# ---------------------------------------------------------------------------
find_package(Qt6 QUIET COMPONENTS Core Gui Widgets Xml Network Svg Charts)
if(NOT Qt6_FOUND)
    message(WARNING
        "Qt6 (Core Gui Widgets Xml Network Svg Charts) not found -- skipping the "
        "Qt ground-segment/tool super-build (messages, gaia, plotter, logplotter, "
        "play, tmtc). Install the Qt6 development packages and re-run CMake to "
        "build them.")
    return()
endif()

# One central compilation of the shared Qt link libraries (IvyQt + pprzlinkQt),
# taken from the Paparazzi sw/ext tree. The helper provides IvyQt from the
# standalone sw/ext/ivy-qt and builds pprzlinkQt directly from the
# sw/ext/pprzlink-qt sources, deliberately bypassing pprzlink-qt's own
# CMakeLists (which hard-codes an add_subdirectory() of its bundled, soon-to-be
# removed nested IvyQt submodule). Both targets are created once and silenced of
# upstream warnings inside the helper; every consumer below links them.
include(${CMAKE_CURRENT_SOURCE_DIR}/ext/pprz_qt_link_libraries.cmake)
pprz_provide_qt_link_libraries()

# ---------------------------------------------------------------------------
# Paparazzi version stamp (sw/include/pprz_version.h)
#
# pprz_version.h turns the build-time token defines GIT_SHA1 / GIT_DESC /
# PPRZ_VER[_MAJOR|_MINOR|_PATCH] into the string macros GIT_VERSION,
# PPRZ_VERSION_DESC and PPRZ_VERSION that the apps print via "-v"/"--version".
# The firmware Makefile (Makefile.ac) derives those values from git; we mirror
# that here -- once, at configure time -- and hand them to every Qt app through
# one INTERFACE target. Non-git (release tarball) builds fall back to UNKNOWN/0
# so the macros are always well-formed. Re-run CMake to refresh the stamp.
# ---------------------------------------------------------------------------
find_package(Git QUIET)
set(PPRZ_GIT_SHA1 "UNKNOWN")
set(PPRZ_GIT_DESC "UNKNOWN")
if(GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} log -1 --pretty=format:%H
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
        OUTPUT_VARIABLE _pprz_sha1
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
    if(_pprz_sha1)
        set(PPRZ_GIT_SHA1 "${_pprz_sha1}")
    endif()
endif()
# paparazzi_version emits the exact "git describe" string the firmware uses.
execute_process(
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../paparazzi_version
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
    OUTPUT_VARIABLE _pprz_desc
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
if(_pprz_desc)
    set(PPRZ_GIT_DESC "${_pprz_desc}")
endif()
# Derive <major>.<minor>.<patch> from the describe string (e.g. "v6.4.0-..." or
# "v7.0_unstable-..."), defaulting any missing numeric part to 0 so the integer
# PPRZ_VERSION_INT macro stays valid.
set(PPRZ_VER_MAJOR 0)
set(PPRZ_VER_MINOR 0)
set(PPRZ_VER_PATCH 0)
if(PPRZ_GIT_DESC MATCHES "^v?([0-9]+)\\.([0-9]+)\\.([0-9]+)")
    set(PPRZ_VER_MAJOR "${CMAKE_MATCH_1}")
    set(PPRZ_VER_MINOR "${CMAKE_MATCH_2}")
    set(PPRZ_VER_PATCH "${CMAKE_MATCH_3}")
elseif(PPRZ_GIT_DESC MATCHES "^v?([0-9]+)\\.([0-9]+)")
    set(PPRZ_VER_MAJOR "${CMAKE_MATCH_1}")
    set(PPRZ_VER_MINOR "${CMAKE_MATCH_2}")
endif()
set(PPRZ_VER "${PPRZ_VER_MAJOR}.${PPRZ_VER_MINOR}.${PPRZ_VER_PATCH}")
message(STATUS "Paparazzi version: ${PPRZ_GIT_DESC} (${PPRZ_VER}, sha ${PPRZ_GIT_SHA1})")

add_library(pprz_version INTERFACE)
target_compile_definitions(pprz_version INTERFACE
    GIT_SHA1=${PPRZ_GIT_SHA1}
    GIT_DESC=${PPRZ_GIT_DESC}
    PPRZ_VER=${PPRZ_VER}
    PPRZ_VER_MAJOR=${PPRZ_VER_MAJOR}
    PPRZ_VER_MINOR=${PPRZ_VER_MINOR}
    PPRZ_VER_PATCH=${PPRZ_VER_PATCH}
)

# ---------------------------------------------------------------------------
# logalizer tools: plotter, logplotter, play
#
# These are single-translation-unit Qt6 apps that share one optimized build
# recipe (Release LTO + section garbage collection + link-time strip) and are
# emitted next to their own sources in sw/logalizer/. They are declared directly
# here -- rather than via a near-empty sw/logalizer/CMakeLists.txt -- because the
# toolchain, Qt6 packages, AUTOMOC/RCC/UIC and the pprzlinkQt/IvyQt libraries are
# already set up above. Only their link libraries differ, so the shared recipe
# lives in the loop and the libraries are attached afterwards.
# ---------------------------------------------------------------------------
foreach(app plotter logplotter play)
    add_executable(${app}
        "logalizer/${app}.cpp"
        "logalizer/${app}.qrc"
    )
    target_include_directories(${app} PRIVATE logalizer ground_segment)
    target_compile_options(${app} PRIVATE
        $<$<CONFIG:Release>:-O2 -DNDEBUG -fdata-sections -ffunction-sections -flto=auto -g0>
    )
    target_link_options(${app} PRIVATE
        $<$<CONFIG:Release>:-Wl,--gc-sections -Wl,-O1 -Wl,-s -flto=auto>
    )
    set_target_properties(${app} PROPERTIES
        INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE
        # Emit the executable beside its sources (e.g. sw/logalizer/play).
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/logalizer"
    )
    install(TARGETS ${app} DESTINATION bin)
endforeach()

# plotter and logplotter render with Qt Charts and decode telemetry via pprzlinkQt.
target_link_libraries(plotter    PRIVATE Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Xml Qt6::Network Qt6::Charts pprzlinkQt pprz_version)
target_link_libraries(logplotter PRIVATE Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Xml Qt6::Network Qt6::Charts pprzlinkQt pprz_version)
# play only re-broadcasts already-formatted log lines on the Ivy bus (no Charts,
# no pprzlink): it reads the few message facts it needs straight from
# messages.xml via Qt6::Xml, so it links just IvyQt.
target_link_libraries(play       PRIVATE Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Xml Qt6::Network IvyQt pprz_version)

# ---------------------------------------------------------------------------
# Remaining Qt applications. Explicit (short) binary directory names keep the
# build tree flat and predictable: each app's executable lands in
# $(QT_BUILD_DIR)/<name>/ regardless of where its sources live under sw/.
# ---------------------------------------------------------------------------
add_subdirectory(ground_segment/tmtc tmtc)
add_subdirectory(simulator simulator)
