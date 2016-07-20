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

import lib.gui as gui

import PyQt5.QtCore as Core
import re


###############################################################################
# [Constants]

DEFAULT_FONT_QCOLOR = gui.generate_qcolor("black")
DEFAULT_BACKGROUND_QCOLOR = gui.generate_qcolor("white")
DEFAULT_LOGGER_FONT_QCOLOR = gui.generate_qcolor("blue")

ALL_MESSAGES_LEVEL = "all"
CUSTOM_MESSAGES_LEVEL = "custom"
MINIMAL_MESSAGES_LEVEL = "minimal"

PROCESS_MESSAGE_TYPE = "process"
APPLICATION_MESSAGE_TYPE = "application"

ERROR_FLAG = "error"
WARNING_FLAG = "warning"
INFO_FLAG = "info"
DEFAULT_FLAG = "default"

BACKGROUNDS_COLORS = {ERROR_FLAG:   "red",
                      WARNING_FLAG: "yellow",
                      INFO_FLAG:    "green",
                      DEFAULT_FLAG: "white"}

FONTS_COLORS = {ERROR_FLAG:   "red",
                WARNING_FLAG: "orange",
                INFO_FLAG:    "blue",
                DEFAULT_FLAG: "black"}

ERROR_REGEX = re.compile(r"[Ee][Rr]{2}[Oo][Rr][ :]")
WARNING_REGEX = re.compile(r"[Ww][Aa][Rr][Nn][Ii][Nn][Gg][ :]")
INFO_REGEX = re.compile(r"[Ii][Nn][Ff][Oo][ :]")


def analyse_log_line(line):
    if ERROR_REGEX.search(line) is not None:
        flag = ERROR_FLAG
    elif WARNING_REGEX.search(line) is not None:
        flag = WARNING_FLAG
    elif INFO_REGEX.search(line) is not None:
        flag = INFO_FLAG
    else:
        flag = DEFAULT_FLAG
    return line, flag


###############################################################################
# [Console class]

class Console(Core.QObject):
    """ Class to define a Console object."""
    def __init__(self, q_text_edit_widget):
        super(Console, self).__init__()

        self.edit = q_text_edit_widget

    def write(self, line, font_color=DEFAULT_FONT_QCOLOR,
              backgroud_color=DEFAULT_BACKGROUND_QCOLOR):

        self.edit.setTextColor(font_color)
        self.edit.setTextBackgroundColor(backgroud_color)
        self.edit.append(line)


###############################################################################
# [LogFilter class]

class LogFilter(object):
    """Class to define a LogFilter object to manage the messages written in the
    integrated console."""
    def __init__(self, init_level_str, init_default_bool, init_info_bool,
                 init_warning_bool, init_error_bool):
        """
        -> 'level' sets what sort of messages to display in the console and is
        linked to the HMI radio-button choice.
        -> 'info', 'warning' and 'error' are booleans to set what logs to
        display in the console and is linked to the HMI check-boxes.
        """
        self.level = init_level_str

        self.default = init_default_bool
        self.info = init_info_bool
        self.warning = init_warning_bool
        self.error = init_error_bool

    def __repr__(self):
        string = "\t| level = {!s:<10} | default = {!s:<5} | info = {!s:<5} |"\
                 " warning = {!s:<5} | error = {!s:<5} |"
        format_string = string.format(self.level, self.default, self.info,
                                      self.warning, self.error)
        return format_string

    def set_level(self, level):
        """
        :param level:
        -> level = 'minimum' (only important messages)
                   'basic' (customized messages)
                   'all' (all messages emitted by Makefiles and compilers).
        """
        self.level = level

    def set_sensitivity_filter(self, default, info, warning, error):
        """
        :param default:
        :param info:
        :param warning:
        :param error:
        -> info, warnings and error are booleans.
        """
        self.default = default
        self.info = info
        self.warning = warning
        self.error = error

    def write_line_decision(self, flag):
        if flag == ERROR_FLAG and self.error:
            return True
        elif flag == WARNING_FLAG and self.warning:
            return True
        elif flag == INFO_FLAG and self.info:
            return True
        elif flag == DEFAULT_FLAG and self.default:
            return True
        else:
            return False
