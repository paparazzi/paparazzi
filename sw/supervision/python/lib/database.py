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


###############################################################################
# [Configuration class]

class Configuration(object):
    """Class to define a Configuration (old aircraft) object."""
    def __init__(self, name, config_id, airframes, targets):
        # Necessary parameters :
        self.name = name
        self.id = config_id
        self.airframes = airframes

        # Not necessary parameters :
        self.settings = []
        self.modules = []
        self.flight_plan = []
        self.radio = []
        self.telemetry = []

        self.color = []

        self.targets = targets

    def __repr__(self):
        string = "| name = {!s:<25} | id = {!s:<3} | airframe = {!s:<65} |" + \
                 "targets = {!s:<250} | color = {!s:<15} | " + \
                 "radio = {!s:<40} | telemetry = {!s:<45} | " + \
                 "flight_plan = {!s:<75} | settings = {!s:<265} | " + \
                 "modules = {!s:<265} |"
        format_string = string.format(self.name, self.id, self.airframes,
                                      [_ for _ in self.targets.values()],
                                      self.color, self.radio,
                                      self.telemetry, self.flight_plan,
                                      self.settings, self.modules)
        return format_string


###############################################################################
# [Set class]

class Set(object):
    """Class to define a Set (old configuration) object."""
    def __init__(self, name):
        self.name = name
        self.configs_names = []

    def __repr__(self):
        string = "| name = {!s:<75} | configs_names = {!s:<680} |"
        format_string = string.format(self.name, self.configs_names)
        return format_string

    def is_config_in_set(self, config):
        return config.id in self.configs_names


###############################################################################
# [Target class]

class Target(object):
    """Class to define a Target object."""
    def __init__(self, name, board, ):
        self.name = name
        self.board = board

        self.defines = None

    def __repr__(self):
        string = "| name = {!s:<15} | board = {!s:<20} |"
        format_string = string.format(self.name, self.board)
        return format_string


###############################################################################
# [Device class]

class Device(object):
    """Class to define a Device (old flash mode) object."""
    def __init__(self, name, variable_name="", variable_value=""):
        self.name = name
        self.variable = (variable_name, variable_value)
        self.boards_regex = []

    def __repr__(self):
        string = "| name = {!s:<25} | variable = {!s:<35} | " + \
                 "boards_regex = {!s:<125} |"
        format_string = string.format(self.name, self.variable,
                                      self.boards_regex)
        return format_string


###############################################################################
# [Session class]

class Session(object):
    """Class to define a Session object."""
    def __init__(self, name, programs):
        self.name = name
        self.programs = programs

    def __repr__(self):
        string = "| name = {!s:<30} | programs = {!s:<1300} |"
        format_string = string.format(self.name,
                                      [_ for _ in self.programs.values()])
        return format_string


###############################################################################
# [Program class]

class Program(object):
    """Class to define a Program (old tool) object."""
    def __init__(self, name, command, options, icon=None, favorite=None, blacklisted=None):
        self.name = name
        self.command = command
        self.options = options
        self.icon = icon
        self.favorite = favorite
        self.blacklisted = blacklisted

    def __repr__(self):
        string = "\t| name = {!s:<30} | command = {!s:<60} | options = {!s:<70} | icon={!s} |"
        format_string = string.format(self.name, self.command, self.options, self.icon)
        return format_string


###############################################################################
# [Cache class]

class Cache(object):
    """Class to define a Cache (old %gconf) object."""
    def __init__(self, last_update_time):
        self.last_update = last_update_time

        self.main_window_shape = (0, 0)
        self.last_set = None
        self.last_configuration = None
        self.last_target = None
        self.last_session = None
        self.last_device = None
