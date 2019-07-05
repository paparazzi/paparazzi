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

import lib.database as db
import lib.environment as env

import xml.etree.ElementTree as Et
import os
import re
import logging
import time

from typing import  List, Dict
import shutil


###############################################################################
# [Constants]

STRINGS_FALSE = ["False", "false", None, "0"]

LOGGER = logging.getLogger("[PARSER]")

XML_EXT = ".xml"
TEST_XML = "test.xml"

# GLOBAL XML TREE REFERENCES :
NAME_REF = "name"
VALUE_REF = "value"
CONSTANT_REF = "constant"
VARIABLE_REF = "variable"
MODE_REF = "mode"
FAVORITE_REF = "favorite"

# REFERENCES AND STRUCTURES OF CONF XML FILES :
# CONF_STRUCTURE = [(node_name, [node_attributes],
#                   [(child_name, [child_attributes],
#                    [children_of_child...]),
#                    (...)
#                    ])
#                   ]

# CACHE :
CACHE = "cache"
CACHE_FILE = "." + CACHE + XML_EXT
DEFAULT_CACHE_FILE = "default_cache" + XML_EXT
DEFAULT_CACHE_PATH = os.path.join("lib", DEFAULT_CACHE_FILE)

LAST_GEOMETRY = "last_geometry"
LAST_GEOMETRY_REF = "system/window/geometry"
LAST_SET = "last_set"
LAST_SET_REF = "data/set"
LAST_CONFIG = "last_config"
LAST_CONFIG_REF = "data/config"
LAST_TARGET = "last target"
LAST_TARGET_REF = "data/target"
LAST_LOG_FILTERS = "last log filters"
LAST_LOG_FILTERS_REF = "system/filters"
LAST_DEVICE = "last device"
LAST_DEVICE_REF = "data/device"
LAST_SESSION = "last session"
LAST_SESSION_REF = "data/session"

# SET :
SET = "conf"
SET_REGEX = re.compile(r"" + SET + "([_-][\w]*)?\.xml$")

CONF_REF = "aircraft"
ID_REF = "ac_id"
AIRFRAME_REF = "airframe"
SETTINGS_REF = "settings"
MODULES_REF = "module"
FP_REF = "flight_plan"
RADIO_REF = "radio"
TELEMETRY_REF = "telemetry"
SETTINGS_MODULES_REF = "settings_modules"
COLOR_REF = "gui_color"

CONFIG_TAG_REF = CONF_REF

SET_STRUCTURE = [(SET, [],
                 [(CONF_REF, [NAME_REF, ID_REF, AIRFRAME_REF, RADIO_REF,
                              TELEMETRY_REF, FP_REF, SETTINGS_REF,
                              SETTINGS_MODULES_REF, COLOR_REF],
                   [])
                  ])
                 ]


# DEVICE :
DEVICE = "flash_modes"
DEVICES_FILE = DEVICE + XML_EXT

BOARD_REF = "board"
BOARDS_REF = "boards"

DEVICE_TAG_REF = MODE_REF

DEVICE_STRUCTURE = [(DEVICE, [],
                    [(MODE_REF, [NAME_REF],
                     [(VARIABLE_REF, [NAME_REF, VALUE_REF],
                      [(BOARDS_REF, [],
                       [(BOARD_REF, [NAME_REF],
                         [])
                        ])
                       ])
                      ])
                     ])
                    ]

# DEFAULT DEVICE :
DEFAULT_DEVICE_NAME = " __Default__ "
DEFAULT_DEVICE = db.Device(DEFAULT_DEVICE_NAME)

# TOOLS
TOOLS = "tools"

# CONTROL_PANEL :
CONTROL_PANEL = "control_panel"
CONTROL_PANEL_FILE = CONTROL_PANEL + XML_EXT

SECTION_REF = "section"
PROGRAM_REF = "program"
SESSION_REF = "session"
COMMAND_REF = "command"
ICON_REF = "icon"
FLAG_REF = "flag"
OPTION_REF = "arg"

SESSION_TAG_REF = "/".join((SECTION_REF, SESSION_REF))

CONTROL_PANEL_STRUCTURE = [(CONTROL_PANEL, [NAME_REF],
                           [(SECTION_REF, [NAME_REF],
                            [(PROGRAM_REF, [NAME_REF, COMMAND_REF],
                             [(OPTION_REF, [FLAG_REF, CONSTANT_REF],
                              [])
                              ])
                             ]),
                            (SECTION_REF, [NAME_REF],
                            [(SESSION_REF, [NAME_REF],
                             [(PROGRAM_REF, [NAME_REF],
                              [(OPTION_REF, [FLAG_REF, CONSTANT_REF],
                               [])
                               ])
                              ])
                             ])
                            ])
                           ]

# DEFAULT SESSIONS :
SIMULATOR_OBJECT = db.Program("Simulator", "sw/simulator/pprzsim-launch",
                              [("-a", "@AIRCRAFT"), ("-t", "@TARGET"),
                               "--boot", "--norc"])
GCS_OBJECT = db.Program("GCS", "sw/ground_segment/cockpit/gcs", [])
SERVER_OBJECT = db.Program("Server", "sw/ground_segment/tmtc/server", ["-n"])
PLAYER_OBJECT = db.Program("Log File Player", "sw/logalizer/play", [])

SIMULATION_NAME = "Simulation"
SIMULATION_SESSION = db.Session(SIMULATION_NAME,
                                {SIMULATOR_OBJECT.name: SIMULATOR_OBJECT,
                                 GCS_OBJECT.name:       GCS_OBJECT,
                                 SERVER_OBJECT.name:    SERVER_OBJECT})
SESSIONS_COMBO_SEP = " - - - - - - - - - - "
REPLAY_NAME = "Replay"
REPLAY_SESSION = db.Session(REPLAY_NAME,
                            {PLAYER_OBJECT.name: PLAYER_OBJECT,
                             SERVER_OBJECT.name: SERVER_OBJECT,
                             GCS_OBJECT.name:    GCS_OBJECT})

# AIRFRAME :
FIRMWARE_REF = "firmware"
TARGET_REF = "target"
BOARD_REF = "board"

TARGET_TAG_REF = "/".join((FIRMWARE_REF, TARGET_REF))

AIRFRAME_STRUCTURE = []  # TODO


###############################################################################
# [Functions]

def full_path_to_filename(path):
    return os.path.basename(path)


def full_to_conf_path(full_path):
    return os.path.relpath(full_path, start=env.PAPARAZZI_CONF)


def filename_to_conf_path(filename):
    return os.path.join(env.PAPARAZZI_CONF, filename)


def find_unused_item_name(dictionary, name):
    new_name = name
    nb = 0
    while new_name in dictionary.keys():
        nb += 1
        new_name = new_name.split("[")[0] + "[" + str(nb) + "]"
    return new_name


###############################################################################
# [Functions] Data management functions

def sorted_sets_names(sets):
    sets_names = [set_object.name for set_object in sets.values()]
    return sorted(sets_names)


def sorted_current_configs_names(configurations, current_set):
    configs_names = [configurations[config_name].name
                     for config_name in current_set.configs_names]
    return sorted(configs_names)


def sorted_current_targets_names(targets):
    target_names = [target.name for target in targets.values()]
    return sorted(target_names)


def sorted_sessions_names(sessions):
    sessions_names = [session.name for session in sessions.values()]
    sorted_names = sorted(sessions_names)
    old_simulation_index = sorted_names.index(SIMULATION_NAME)
    sorted_names.insert(0, sorted_names.pop(old_simulation_index))
    old_replay_index = sorted_names.index(REPLAY_NAME)
    sorted_names.insert(1, sorted_names.pop(old_replay_index))
    sorted_names.insert(2, SESSIONS_COMBO_SEP)
    return sorted_names


def sorted_current_programs_names(current_session):
    programs_names = [program.name for program
                      in current_session.programs.values()]
    return sorted(programs_names)


def sorted_current_devices_names(devices):
    devices_names = [device.name for device in devices]
    return sorted(devices_names)


def sorted_tools_names(tools):
    tools_names = [tool.name for tool in tools.values()]
    return sorted(tools_names)


###############################################################################
# [Functions] Load cache from '.cache' file

def parse_cache_file(mode, cache_file,
                     geometry_str=None, set_str=None, config_str=None,
                     target_str=None, log_filters_str=None,
                     device_str=None, session_str=None):
    """
    :param mode:
    :param cache_file:
    :param geometry_str:
    :param set_str:
    :param config_str:
    :param target_str:
    :param log_filters_str:
    :param device_str:
    :param session_str:
    -> Scan 'cache_file' to get tags by name reference.
    -> Read values of tags in 'r' mode.
    -> Write new values of tags in 'w' mode.
    -> Except an incorrect XML format and raise ERROR.
    """
    cache_dict = {}
    try:
        try:
            cache_tree = Et.parse(cache_file)
        except FileNotFoundError:
            cache_tree = Et.parse(DEFAULT_CACHE_PATH)

        last_geometry_tag = cache_tree.find(LAST_GEOMETRY_REF)
        last_set_tag = cache_tree.find(LAST_SET_REF)
        last_config_tag = cache_tree.find(LAST_CONFIG_REF)
        last_target_tag = cache_tree.find(LAST_TARGET_REF)
        last_log_filters_tag = cache_tree.find(LAST_LOG_FILTERS_REF)
        last_device_tag = cache_tree.find(LAST_DEVICE_REF)
        last_session_tag = cache_tree.find(LAST_SESSION_REF)

        last_tags_list = [last_geometry_tag, last_set_tag,
                          last_config_tag, last_target_tag,
                          last_log_filters_tag, last_device_tag,
                          last_session_tag]
        if mode == "r":
            for key, tag in zip([LAST_GEOMETRY, LAST_SET,
                                 LAST_CONFIG, LAST_TARGET,
                                 LAST_LOG_FILTERS, LAST_DEVICE,
                                 LAST_SESSION],
                                last_tags_list):
                cache_dict[key] = tag.get(VALUE_REF)
            return cache_dict

        elif mode == "w":
            for tag, string in zip(last_tags_list,
                                   [geometry_str, set_str,
                                    config_str, target_str,
                                    log_filters_str, device_str,
                                    session_str]):
                tag.set(VALUE_REF, string)
            cache_tree.write(cache_file)

    except Et.ParseError as msg:
            LOGGER.error("ERROR in syntax of XML file : '%s'. "
                         "Original message : '%s'.", cache_file, msg)


def load_cache():
    """
    -> Scan the 'python' current directory to find the '.cache.xml' file.
    -> Parse the cache file withe the 'w' mode to load the last values.
    -> Except multiple cache files and raise an ERROR if so.
    """
    cache_file = None
    for root, dirs, files in os.walk("./"):
        for file in files:
            ext = os.path.splitext(file)[1]
            if ext == XML_EXT and file == CACHE_FILE:
                cache_file = os.path.join(root, file)
                break
    if cache_file is not None:
        cache = parse_cache_file("r", cache_file)
    else:
        cache = parse_cache_file("r", DEFAULT_CACHE_PATH)
        cache_file = CACHE_FILE

        LOGGER.error("No cache file found ! Default cache '%s' loaded.",
                     DEFAULT_CACHE_PATH)
        print("No cache file found ! Default cache '%s' loaded." %
              DEFAULT_CACHE_PATH)

    if logging.DEBUG:
        LOGGER.debug("Cache dictionary :\n%s", cache)
    return cache, cache_file

def delete_cache():
    """
    -> Scan the 'python' current directory to find the '.cache.xml' file.
    -> Remove if found
    """
    cache_file = None
    for root, dirs, files in os.walk("./"):
        for file in files:
            ext = os.path.splitext(file)[1]
            if ext == XML_EXT and file == CACHE_FILE:
                cache_file = os.path.join(root, file)
                break
    if cache_file is not None:
        cache = os.remove(cache_file)
        LOGGER.debug("Deleting cache file '%s'\n", cache_file)
    else:
        LOGGER.debug("No cache to delete\n")


###############################################################################
# [Functions] Load files to init HMI

def load_init_files(conf_path):
    """
    :param conf_path:
    -> Scan the conf directory to find the configuration files : all files
    matching with '*conf*.xml', 'control_panel.xml' and 'flash_modes.xml'.
    -> Show the result of scan if DEBUG mode is on (main.py)
    """
    conf_files, devices_files = [], []
    cp_file = None
    
    cp_path = conf_path + "/" + CONTROL_PANEL + XML_EXT
    if os.path.exists(cp_path):
        cp_file = cp_path
    else:
        raise Exception("%s not found!"% conf_path)

    tools_path = conf_path + "/" + TOOLS
    if not os.path.exists(tools_path):
        raise Exception("%s not found!" % tools_path)
    
    for root, dirs, files in os.walk(conf_path):
        for file in files:
            ext = os.path.splitext(file)[1]
            if ext == XML_EXT:
                xml_file = os.path.join(root, file)
                if SET_REGEX.search(file) is not None \
                        and file != "%gconf.xml"\
                        and xml_file != env.PAPARAZZI_CONF+"/conf.xml":
                    conf_files.append(xml_file)
                elif file == CONTROL_PANEL_FILE and "airframes" not in root:
                    pass
                elif file == DEVICES_FILE:
                    devices_files.append(xml_file)

    result = "{} startup files found."  # exclude control_panel.xml
    files_nb = sum((len(conf_files), len(devices_files)))
    info = result.format(files_nb)

    if logging.DEBUG:
        LOGGER.debug("'conf' files :")
        for file in conf_files:
            LOGGER.debug(file)
        LOGGER.debug("'devices' file(s) :")
        for file in devices_files:
            LOGGER.debug(file)
        LOGGER.debug("'control_panel' file(s) :")
        LOGGER.debug(cp_file)
    return conf_files, cp_file, tools_path, devices_files, info


###############################################################################
# [Functions] Load sets and configurations from 'conf' files

def set_config_details(config_object, tag):
    """
    :param config_object:
    :param tag:
    -> Parse values of a configuration in the given XML tag element.
    -> Fills attributes of the given configuration object.
    """
    variables = [[] for _ in range(6)]
    keys = [SETTINGS_REF, SETTINGS_MODULES_REF, FP_REF, RADIO_REF,
            TELEMETRY_REF, COLOR_REF]
    for i, key in zip(range(6), keys):
        items = tag.get(key)
        if items:
            for item in items.split(" "):
                variables[i].append(item)

    config_object.settings = variables[0]
    config_object.modules = variables[1]
    config_object.flight_plan = variables[2]
    config_object.radio = variables[3]
    config_object.telemetry = variables[4]
    config_object.color = variables[5]


def parse_targets(airframe_file):
    """
    :param airframe_file:
    -> Parse values of a target in the airframe file given.
    -> Except an incorrect XML format and raise ERROR.
    """
    available_targets = {}
    try:
        airframe_tree = Et.parse(filename_to_conf_path(airframe_file))
        targets_tags = airframe_tree.findall(TARGET_TAG_REF)
        for target_tag in targets_tags:
            name = target_tag.get(NAME_REF)
            board = target_tag.get(BOARD_REF)
            target_object = db.Target(name, board)
            available_targets[target_object.name] = target_object

    except Et.ParseError as msg:
            LOGGER.error("ERROR in syntax of XML file : '%s'. "
                         "Original message : '%s'.", airframe_file, msg)
    except (FileNotFoundError, IOError) as msg:
            LOGGER.error("ERROR file '%s' not found or IO error"
                         "Original message : '%s'.", airframe_file, msg)
    return available_targets


def parse_conf_files(set_files):
    """
    :param set_files:
    -> Parse values of a target in the airframe file given.
    -> Except an incorrect XML format and raise ERROR.
    """
    configs, sets = {}, {}
    for set_file in set_files:
        try:
            set_name = full_to_conf_path(set_file)
            set_object = db.Set(set_name)
            set_tree = Et.parse(set_file)
            config_tags = set_tree.findall(CONFIG_TAG_REF)
            for tag in config_tags:
                config_name = tag.get(NAME_REF)
                config_id = tag.get(ID_REF)
                config_airframes = tag.get(AIRFRAME_REF).split(" ")
                config_targets = parse_targets(config_airframes[0])
                config_object = db.Configuration(config_name, config_id,
                                                 config_airframes,
                                                 config_targets)
                if config_object.id not in set_object.configs_names:
                    set_config_details(config_object, tag)
                    configs[config_object.name] = config_object
                    set_object.configs_names.append(config_object.name)
                else:
                    LOGGER.error("'%s' configuration can't be loaded in "
                                 "'%s' set because ID already exists !",
                                 config_object.name, set_object.name)
            sets[set_name] = set_object

        except Et.ParseError as msg:
            LOGGER.error("ERROR in syntax of XML file : '%s'. "
                         "Original message : '%s'.", set_file, msg)
    return configs, sets


def load_sets_and_configurations(set_files):
    """
    :param set_files:
    -> Parse the sets and the configurations from the '*conf*.xml' files.
    -> Show the result of scan if DEBUG mode is on (main.py)
    """
    configurations, sets = parse_conf_files(set_files)
    result = "{} sets and {} configurations found."
    sets_nb, configs_nb = len(sets), len(configurations)
    info = result.format(sets_nb, configs_nb)

    if logging.DEBUG:
        sets_str = "Sets :\n"
        for set_object in sets.values():
            sets_str += str(set_object) + "\n"
        LOGGER.debug(sets_str)
        configs_str = "Configurations :\n"
        for config_object in configurations.values():
            configs_str += str(config_object) + "\n"
        LOGGER.debug(configs_str)
    return sets, configurations, info


def save_configuration(conf_file, config_object):
    """
    :param conf_file:
    :param config_object:
    -> Set the new values of the configuration object given into the
    corresponding XML file.
    """
    conf_file_path = filename_to_conf_path(conf_file)

    conf_tree = Et.parse(conf_file_path)
    config_tags = conf_tree.findall(CONFIG_TAG_REF)
    for tag in config_tags:
        if tag.get(NAME_REF) == config_object.name:
            attributes = [AIRFRAME_REF, SETTINGS_REF,
                          SETTINGS_MODULES_REF, FP_REF,
                          RADIO_REF, TELEMETRY_REF]
            new_values = [config_object.airframes, config_object.settings,
                          config_object.modules, config_object.flight_plan,
                          config_object.radio, config_object.telemetry]
            for attribute, values in zip(attributes, new_values):
                string = " ".join(values)
                tag.set(attribute, string)
    conf_tree.write(conf_file_path)


###############################################################################
# [Functions] Load sets devices from 'flash_modes' file

def parse_devices_file(devices_file):
    """
    :param devices_file:
    -> Parse values of a device in the 'flash_modes' file given.
    -> Except an incorrect XML format and raise ERROR.
    """
    devices = {}
    try:
        devices_tree = Et.parse(devices_file)
        devices_tags = devices_tree.findall(DEVICE_TAG_REF)
        for device_tag in devices_tags:
            device_name = device_tag.get(NAME_REF)
            variable_tag = device_tag.find(VARIABLE_REF)
            variable_name = variable_tag.get(NAME_REF)
            variable_value = variable_tag.get(VALUE_REF)
            device_object = db.Device(device_name,
                                      variable_name, variable_value)
            for board_tag in device_tag.find(BOARDS_REF):
                board_name = board_tag.get(NAME_REF)
                device_object.boards_regex.append(board_name)
            devices[device_name] = device_object
    except Et.ParseError as msg:
            LOGGER.error("ERROR in syntax of XML file : '%s'. "
                         "Original message : '%s'.", devices_file, msg)
    return devices


def load_devices(devices_file):
    """
    :param devices_file:
    -> Parse the devices from the 'flash_modes.xml' files.
    -> Show the result of scan if DEBUG mode is on (main.py)
    """
    devices = parse_devices_file(devices_file)
    default_object = db.Device("Default")
    devices[default_object.name] = default_object

    devices[DEFAULT_DEVICE.name] = DEFAULT_DEVICE

    result = "{} devices found."
    devices_nb = len(devices)
    info = result.format(devices_nb)

    if logging.DEBUG:
        devices_str = "Devices :\n"
        for device_object in devices.values():
            devices_str += str(device_object) + "\n"
        LOGGER.debug(devices_str)

    return devices, info


###############################################################################
# [Functions] Load tools and sessions from 'control_panel' file

def parse_arg_option(option_tag):
    """
    :param option_tag:
    -> Fill an option triplet (flag, argument type : 'constant' or 'variable',
    value of the argument) from an option tag in the XML tree.
    """
    option_flag = option_tag.get(FLAG_REF)
    constant = option_tag.get(CONSTANT_REF)
    variable = option_tag.get(VARIABLE_REF)
    if constant is not None:
        option = (option_flag, constant)
    elif variable is not None:
        option = (option_flag, variable)
    else:
        option = option_flag
    return option


def parse_tools(tools_path):
    """
    :param tools_path:
    -> Parse all tools files in the 'tools_path' directory.
    -> Except an incorrect XML format and raise ERROR.
    """
    tools = {}
    blacklisted_tools = []
    blacklist = tools_path + "/" + "blacklisted"
    if os.path.exists(blacklist):
        with open(blacklist, 'r') as blacklist_fic:
            for line in blacklist_fic:
                line = line.strip()
                if line != "" and line[0] != "#":
                    blacklisted_tools.append(line)

    for file in os.listdir(tools_path):
        if file.endswith(".xml"):
            file_path = tools_path + "/" + file
            try:
                tree = Et.parse(file_path)
                tool_tag = tree.getroot()
                if tool_tag.tag == PROGRAM_REF:
                    tool_name = tool_tag.get(NAME_REF)
                    tool_command = tool_tag.get(COMMAND_REF)
                    icon = tool_tag.get(ICON_REF)
                    fav = tool_tag.get(FAVORITE_REF)
                    favorite = fav if fav is None else (fav not in STRINGS_FALSE)

                    options = []
                    for option_tag in tool_tag:
                        option = parse_arg_option(option_tag)
                        options.append(option)
                    blacklisted = True if tool_name in blacklisted_tools else False
                    tool_object = db.Program(tool_name, tool_command, options, icon, favorite=favorite, blacklisted=blacklisted)
                    tools[tool_name] = tool_object

            except Et.ParseError as msg:
                    LOGGER.error("ERROR in syntax of XML file : '%s'. "
                                 "Original message : '%s'.", file, msg)
    return tools


def parse_sessions(cp_file, tools):
    """
    :param cp_file:
    :param tools:
    -> Parse all sessions in the 'control_panel.xml' file given.
    -> Except an incorrect XML format and raise ERROR.
    """
    sessions = {}
    try:
        cp_tree = Et.parse(cp_file)
        sessions_tags = cp_tree.findall(SESSION_TAG_REF)
        for session_tag in sessions_tags:
            session_name = session_tag.get(NAME_REF)
            session_programs = {}
            for program_tag in session_tag:
                program_name = program_tag.get(NAME_REF)
                command = tools[program_name].command
                options = []
                for option_tag in program_tag:
                    option = parse_arg_option(option_tag)
                    options.append(option)
                program_object = db.Program(program_name, command, options)
                session_programs[program_name] = program_object
            session_object = db.Session(session_name, session_programs)
            sessions[session_name] = session_object

    except Et.ParseError as msg:
            LOGGER.error("ERROR in syntax of XML file : '%s'. "
                         "Original message : '%s'.", cp_file, msg)
    return sessions


def load_sessions_and_programs(cp_file, tools_path):
    """
    :param cp_file:
    :param tools_path:
    -> Parse the tools and sessions from the 'tools:*xml' files.
    -> Add the default sessions 'simulation' & replay.
    -> Show the result of scan if DEBUG mode is on (main.py)
    """
    tools = parse_tools(tools_path)
    sessions = parse_sessions(cp_file, tools)

    sessions[SIMULATION_SESSION.name] = SIMULATION_SESSION
    sessions[REPLAY_SESSION.name] = REPLAY_SESSION

    result = "{} tools and {} sessions found."
    tools_nb, sessions_nb = len(tools), len(sessions)
    info = result.format(tools_nb, sessions_nb)

    if logging.DEBUG:
        tools_str = "Tools :\n"
        for tool_object in tools.values():
            tools_str += str(tool_object) + "\n"
        LOGGER.debug(tools_str)
        sessions_str = "Sessions :\n"
        for session_object in sessions.values():
            sessions_str += str(session_object) + "\n"
        LOGGER.debug(sessions_str)

    return tools, sessions, info


def save_session(cp_file, session_object):
    """
    :param cp_file:
    :param session_object:
    -> Set the new values of the session object given into the
    corresponding XML file.
    """
    cp_file_path = filename_to_conf_path(cp_file)

    cp_tree = Et.parse(cp_file_path)
    cp_tags = cp_tree.findall(SESSION_TAG_REF)
    for session_tag in cp_tags:
        if session_tag.get(NAME_REF) == session_object.name:
            session_tag.clear()
            session_tag.set(NAME_REF, session_object.name)
            for program in session_object.programs.values():
                program_tag = Et.SubElement(session_tag, PROGRAM_REF,
                                            {NAME_REF: program.name})
                for option in program.options:
                    attributes = {}
                    if type(option) is tuple:
                        attributes[FLAG_REF] = option[0]
                        attributes[CONSTANT_REF] = option[1]
                    else:
                        attributes[FLAG_REF] = option
                    _option_tag = Et.SubElement(program_tag, OPTION_REF,
                                                attributes)
    cp_tree.write(cp_file_path)


###############################################################################
# [Data class]

class Data(object):
    """Class to manage a Data meta-object that gathers all the useful data
    parsed in the various XML files of the /conf directory."""
    def __init__(self, conf_path):
        self.conf_path = conf_path

        self.cache_file = None
        self.conf_files = []
        self.devices_file = None
        self.cp_file = None
        self.tools_path = None

        self.cache = {}
        self.configurations = {}
        self.sets = {}
        self.devices = {}
        self.tools = {}
        self.sessions = {}

        self.init_data()

###############################################################################
# [Data methods] Init data methods

    def init_data(self):
        init = time.clock()

        self.load_cache()
        self.load_conf_files()
        self.load_sets_and_configs()
        self.load_devices()
        self.load_sessions_and_programs()

        LOGGER.info("All data loaded in : %s seconds.\n",
                    time.clock() - init)

    def load_cache(self):
        LOGGER.info("Loading cache...")
        self.cache, self.cache_file = load_cache()
        LOGGER.info("Cache loaded.\n")

    def load_conf_files(self):
        LOGGER.info("Scanning current directory...")
        self.conf_files, self.cp_file, self.tools_path, self.devices_file, load_info = \
            load_init_files(self.conf_path)
        LOGGER.debug(load_info)
        LOGGER.info("End of scan.\n")

    def load_sets_and_configs(self):
        LOGGER.info("Loading sets and configurations...")
        self.sets, self.configurations, \
            load_info = load_sets_and_configurations(self.conf_files)
        LOGGER.debug(load_info)
        LOGGER.info("Sets and configurations loaded.\n")

    def load_devices(self):
        LOGGER.info("Loading devices...")
        if len(self.devices_file) == 1:
            self.devices, load_info = load_devices(self.devices_file[0])
            LOGGER.debug(load_info)
            LOGGER.info("Devices loaded.\n")
        else:
            LOGGER.error("Multiple '%s' XML files !", DEVICE)

    def load_sessions_and_programs(self):
        LOGGER.info("Loading programs and sessions...")
        if self.cp_file is not None and self.tools_path is not None:
            self.tools, self.sessions, \
                load_info = load_sessions_and_programs(self.cp_file, self.tools_path)
            LOGGER.debug(load_info)
            LOGGER.info("Programs and sessions loaded.\n")
        else:
            LOGGER.error("ERROR : control_panel.xml or tools.xml not found!")
