# This confidential and proprietary software may be used only as
# authorised by a licensing agreement from ARM Limited
#   (C) COPYRIGHT 2010 - 2013 ARM Limited
#       ALL RIGHTS RESERVED
# The entire notice above must be reproduced on all authorised
# copies and copies may only be made to the extent permitted
# by a licensing agreement from ARM Limited.
#

set (CMAKE_SYSTEM_NAME Linux)
set (CMAKE_SYSTEM_PROCESSOR arm)

set (CMAKE_CXX_COMPILER "${TOOLCHAIN_ROOT}g++")
set (CMAKE_C_COMPILER "${TOOLCHAIN_ROOT}gcc")

set (CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_ROOT})

set (CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set (CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
set (CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
