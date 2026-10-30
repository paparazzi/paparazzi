# ---------------------------------------------------------------------------
# pprz_qt_link_libraries.cmake -- shared Paparazzi Qt6 link libraries (IvyQt + pprzlinkQt)
#
# Provides a single CMake function, pprz_provide_qt_link_libraries(), that
# creates the two shared Qt link libraries used by every native Qt6 application
# in the Paparazzi ground segment:
#
#   * IvyQt      -- Ivy software-bus binding (sw/ext/ivy-qt)
#   * pprzlinkQt -- PprzLink message encode/decode over Ivy (sw/ext/pprzlink-qt)
#
# Both are built exactly once from the canonical sources under sw/ext so that
# the logalizer tools (plotter, logplotter, play) and the tmtc / simulator
# super-build (see sw/CMakeLists.txt), as well as the separately-built GCS
# "cockpit" (sw/ground_segment/cockpit, which includes this helper through a
# relative path), all link the very same libraries.
#
# IvyQt is provided straight from the standalone sw/ext/ivy-qt, whose own
# listfile defines a clean IvyQt target (it carries no nested submodule).
#
# pprzlinkQt is built *directly from the sw/ext/pprzlink-qt sources here*,
# deliberately NOT running pprzlink-qt's own CMakeLists.txt: that listfile
# hard-codes add_subdirectory(IvyQt ${CMAKE_BINARY_DIR}/ivyqt) for its bundled
# (being phased out, frequently uninitialised) nested IvyQt submodule, which
# would clash with the canonical IvyQt provided above on both the IvyQt target
# name and the shared `ivyqt` binary directory. The recipe below otherwise
# mirrors pprzlink-qt/CMakeLists.txt exactly.
#
# This helper lives in the main Paparazzi repository (not in a submodule) so it
# survives `git submodule update`. The pinned upstream sources are never edited;
# their own warnings are silenced (-w) so the consumers' strict -Wall -Wextra
# builds stay warning-free.
# ---------------------------------------------------------------------------

# Directory of THIS file (sw/ext), captured at include() time so the source
# locations resolve correctly no matter which project includes the helper or
# from where it is invoked.
set(PPRZ_QT_LINK_HELPER_DIR "${CMAKE_CURRENT_LIST_DIR}")

function(pprz_provide_qt_link_libraries)
    # Idempotent: if both targets already exist (e.g. a parent scope provided
    # them, or the helper was invoked twice), there is nothing to do.
    if(TARGET pprzlinkQt AND TARGET IvyQt)
        return()
    endif()

    set(_ivy_dir  "${PPRZ_QT_LINK_HELPER_DIR}/ivy-qt")
    set(_pprz_dir "${PPRZ_QT_LINK_HELPER_DIR}/pprzlink-qt")

    if(NOT EXISTS "${_ivy_dir}/CMakeLists.txt")
        message(FATAL_ERROR
            "pprz_qt_link_libraries.cmake: IvyQt sources were not found at \"${_ivy_dir}\". "
            "Run `git submodule update --init` under sw/ext to fetch ivy-qt.")
    endif()
    if(NOT EXISTS "${_pprz_dir}/src/Message.cpp")
        message(FATAL_ERROR
            "pprz_qt_link_libraries.cmake: pprzlinkQt sources were not found at "
            "\"${_pprz_dir}\". Run `git submodule update --init` under sw/ext "
            "to fetch pprzlink-qt.")
    endif()

    # Both libraries expose Q_OBJECT types and therefore need the meta-object
    # compiler (AUTOMOC). ivy-qt's listfile enables it for IvyQt itself; the
    # pprzlinkQt target created below gets it set explicitly.
    set(CMAKE_AUTOMOC ON)

    # Qt packages required by the two libraries. Calling find_package again is
    # harmless (idempotent): the sw/ super-build has already discovered Qt6,
    # while the cockpit build reaches this helper before its own find_package,
    # so doing it here keeps the helper self-sufficient regardless of include
    # order. Mirror the subprojects' optional QT_DIR override.
    if(DEFINED ENV{QT_DIR})
        set(Qt6_DIR "$ENV{QT_DIR}")
    endif()
    find_package(QT NAMES Qt6 REQUIRED COMPONENTS Core Xml Network)
    find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Xml Network)

    # ---- IvyQt -------------------------------------------------------------
    # An explicit binary directory is mandatory because the sources live outside
    # the including project's source tree. The `ivyqt` name is the one that
    # pprzlink-qt's bypassed listfile would otherwise have used, kept here so the
    # build layout is unchanged.
    if(NOT TARGET IvyQt)
        add_subdirectory("${_ivy_dir}" "${CMAKE_BINARY_DIR}/ivyqt")
    endif()

    # ---- pprzlinkQt --------------------------------------------------------
    # Defined here from sources, bypassing pprzlink-qt/CMakeLists.txt (see the
    # file header for why). Mirrors that listfile's target otherwise.
    if(NOT TARGET pprzlinkQt)
        add_library(pprzlinkQt
            "${_pprz_dir}/src/FieldValue.cpp"
            "${_pprz_dir}/src/Message.cpp"
            "${_pprz_dir}/src/MessageDefinition.cpp"
            "${_pprz_dir}/src/MessageDictionary.cpp"
            "${_pprz_dir}/src/MessageField.cpp"
            "${_pprz_dir}/src/MessageFieldTypes.cpp"
            "${_pprz_dir}/src/IvyQtLink.cpp"
            "${_pprz_dir}/include/pprzlinkQt/IvyQtLink.h"
        )

        set_target_properties(pprzlinkQt PROPERTIES AUTOMOC ON)

        target_include_directories(pprzlinkQt PUBLIC
            "$<BUILD_INTERFACE:${_pprz_dir}/include/>")

        # IvyQt is a PRIVATE dependency: the public header IvyQtLink.h only
        # forward-declares `class IvyQt;`, so consumers never need IvyQt's
        # headers, while pprzlinkQt's own translation units (IvyQtLink.cpp
        # includes <IvyQt/ivyqt.h>) do -- which a PRIVATE link supplies. Because
        # pprzlinkQt is a static library, CMake still propagates IvyQt to final
        # consumers for the link step.
        target_link_libraries(pprzlinkQt PRIVATE
            Qt${QT_VERSION_MAJOR}::Core
            Qt${QT_VERSION_MAJOR}::Xml
            Qt${QT_VERSION_MAJOR}::Network
            IvyQt)

        target_compile_definitions(pprzlinkQt PRIVATE PPRZLINKQT_LIBRARY)
    endif()

    # The pinned upstream sources must compile cleanly inside the consumers'
    # strict -Wall -Wextra (and, in the cockpit, -Werror) builds, so silence
    # their own diagnostics. Safe no-op if a target somehow does not exist.
    foreach(_pprz_lib IvyQt pprzlinkQt)
        if(TARGET ${_pprz_lib})
            target_compile_options(${_pprz_lib} PRIVATE -w)
        endif()
    endforeach()
endfunction()
