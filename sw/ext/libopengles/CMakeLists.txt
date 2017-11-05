#
# This confidential and proprietary software may be used only as
# authorised by a licensing agreement from ARM Limited
#   (C) COPYRIGHT 2013 ARM Limited
#       ALL RIGHTS RESERVED
# The entire notice above must be reproduced on all authorised
# copies and copies may only be made to the extent permitted
# by a licensing agreement from ARM Limited.
#

cmake_minimum_required (VERSION 2.8.0)

# Decide which target are we building for
if (TARGET)
    if((${TARGET} STREQUAL "arm") OR (${TARGET} STREQUAL "emulator"))
        message ("\nBuilding for ${TARGET}\n")
    else()
        MESSAGE(FATAL_ERROR "Target ${TARGET} is not valid. Choices are \"emulator\" and \"arm\".")
    endif()
else(TARGET)
    message ("\nTARGET was not set. Choices are \"emulator\" and \"arm\".")
    message ("Example usage: cmake .. -DTARGET=emulator")
    message ("Now using [arm] as default.\n")
    set (TARGET "arm")
endif(TARGET)

if(${TARGET} STREQUAL "arm")
    # If the TOOLCHAIN_ROOT environment variable is set, use it, otherwise
    # try to find the compiler on the path.
    if (DEFINED ENV{TOOLCHAIN_ROOT})
        message ("Using the environment variable TOOLCHAIN_ROOT")
        message ("as a path to search to for your toolchain.")
        message ("If you don't want to use the environment variable you must clear it.")
        set (TOOLCHAIN_ROOT $ENV{TOOLCHAIN_ROOT})
    else()
        message ("The environment variable TOOLCHAIN_ROOT is not set.")
        message ("If you set TOOLCHAIN_ROOT we will use it as a path to search to for your toolchain.")
        message ("Using arm-none-linux-gnueabi- as the default toolchain.")
        set (TOOLCHAIN_ROOT arm-none-linux-gnueabi-)
    endif()

    set (CMAKE_TOOLCHAIN_FILE arm-linux.cmake)
endif()

project(opengles_sdk)

# Don't build an RPATH into the libraries
set(CMAKE_SKIP_RPATH TRUE)

# Use relative paths in the generated projects
set(CMAKE_USE_RELATIVE_PATHS TRUE)

# Add the DEBUG definition if not building in release mode
if(NOT ${CMAKE_BUILD_TYPE} STREQUAL "Release")
    add_definitions(-DDEBUG)
endif()

# Add the Khronos header files and the simple-framework as a shared include location
include_directories(inc)
include_directories(simple_framework/inc)
include_directories(simple_framework/inc/models)

# Set the install prefix per target.
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/bin_${TARGET})

if(${TARGET} STREQUAL "emulator")
    if (WIN32)
        # Win32 settings
        set(GLES_LIB "libGLESv2.lib")
        set(EGL_LIB "libEGL.lib")
        add_definitions(-DWIN32)
        add_definitions(-D_WIN32)

        # Disable Visual C++ warnings for unsafe functions (Warning C4996)
        add_definitions(-D_CRT_SECURE_NO_WARNINGS)
        add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
    else()
        # Linux settings
        set (GLES_LIB "libGLESv2.so")
        set (EGL_LIB "libEGL.so")
        set (PLATFORM_LIBRARIES X11 m)
    endif()

    set(LIB_DIR "lib/emulator_x64" "lib/emulator_x86")
    find_library(OPENGLES_LIBRARY_EGL NAMES ${EGL_LIB} PATHS ${LIB_DIR} NO_DEFAULT_PATH)
    find_library(OPENGLES_LIBRARY_GLES NAMES ${GLES_LIB} PATHS ${LIB_DIR} NO_DEFAULT_PATH)
    # -rpath-link required since libEGL.so/libGLESv2.so also link against other libraries and they are likely not in LD_LIBRARY_PATH.
    set (OPENGLES_LIBRARIES "-Wl,-rpath-link,${PROJECT_SOURCE_DIR}/lib/emulator_x64" "-Wl,-rpath-link,${PROJECT_SOURCE_DIR}/lib/emulator_x86" ${OPENGLES_LIBRARY_EGL} ${OPENGLES_LIBRARY_GLES})

else(${TARGET} STREQUAL "arm")
    # Linux on ARM
    set (GLES_LIB "libGLESv2.so")
    set (EGL_LIB "libEGL.so")

    set(DUMMY_MALI_LIBRARIES GLESv2 EGL)
    add_subdirectory(lib/arm)
    set (OPENGLES_LIBRARIES ${DUMMY_MALI_LIBRARIES})
    include_directories(simple_framework/inc/mali)
    set (PLATFORM_LIBRARIES m)
endif()

add_subdirectory(simple_framework)
add_subdirectory(samples)
