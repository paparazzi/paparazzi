# Paparazzi center utilities
#
# Copyright (C) 2016 ENAC, Florian BITARD (intern student)
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.

###############################################################################
# [Imports]

import os
from sys import platform as os_name
import logging


###############################################################################
# [Constants]

LOGGER = logging.getLogger("[ENV]")
OS = os_name

SRC_NAME = "PAPARAZZI_SRC"
HERE_TO_SRC = "../../../.."
CONF_NAME = "PAPARAZZI_CONF"
HOME_TO_CONF = "conf"
HOME_NAME = "PAPARAZZI_HOME"
HOME_TO_VAR = "var"
IVY_BUS_NAME = "IVY_BUS"


def get_src_dir(src_name):
    current_dir = os.path.dirname(os.path.abspath(__file__))
    src_dir = os.path.normpath(os.path.join(current_dir, HERE_TO_SRC))
    return os.getenv(src_name, src_dir)

PAPARAZZI_SRC = get_src_dir(SRC_NAME)
LOGGER.debug("%s=%s", SRC_NAME, PAPARAZZI_SRC)


def get_home_dir(home_name):
    return os.getenv(home_name, get_src_dir(SRC_NAME))

PAPARAZZI_HOME = get_home_dir(HOME_NAME)
LOGGER.info("%s=%s", HOME_NAME, PAPARAZZI_HOME)


def get_conf_dir(conf_name):
    conf_dir = os.path.join(PAPARAZZI_HOME, HOME_TO_CONF)
    if os.path.exists(conf_dir):
        return os.getenv(conf_name, conf_dir)
    else:
        LOGGER.error("'%s' directory doesn't exist !", conf_dir)

PAPARAZZI_CONF = get_conf_dir(CONF_NAME)


def get_ivy_bus(ivy_bus_name):
    supposed_ivy_bus = os.getenv(ivy_bus_name)
    if supposed_ivy_bus is not None:
        return supposed_ivy_bus
    elif OS == 'linux':
        return "127.255.255.255:2010"
    elif OS == 'linux2':
        return "127.255.255.255:2010"
    elif OS == 'darwin':
        return "224.5.6.7:8910"
    LOGGER.error("Unknown Ivy bus for the current OS !")

IVY_BUS = get_ivy_bus(IVY_BUS_NAME)
LOGGER.debug("%s=%s", HOME_NAME, PAPARAZZI_HOME)

RUN_VERSION_EXE_NAME = "paparazzi_version"
RUN_VERSION_EXE = os.path.join(PAPARAZZI_HOME, RUN_VERSION_EXE_NAME)

BUILD_VERSION_FILE_NAME = "build_version.txt"
BUILD_VERSION_FILE = os.path.join(PAPARAZZI_HOME, HOME_TO_VAR,
                                  BUILD_VERSION_FILE_NAME)
