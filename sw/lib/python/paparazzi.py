#!/usr/bin/env python

from __future__ import print_function

import glob

from collections import namedtuple
from os import path, getenv, walk
from fnmatch import fnmatch
#from subprocess import call
import subprocess

import lxml.etree as ET

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PAPARAZZI_SRC   = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../')))
PAPARAZZI_HOME  = getenv("PAPARAZZI_HOME", PAPARAZZI_SRC)

# Directories
conf_dir        = path.join(PAPARAZZI_HOME, "conf/")

firmwares_dir   = path.join(conf_dir, "modules/firmwares/")
modules_dir     = path.join(conf_dir, "modules/")
airframes_dir   = path.join(conf_dir, "airframes/")
boards_dir      = path.join(conf_dir, "modules/boards/")

flight_plan_dir = path.join(conf_dir, "flight_plans/")

# Structures
PprzModule = namedtuple("PprzModule", "description defines configures")

# List Of Stuff

def get_list_of_conf_files(exclude_backups = 1):
    conf_files = []
    pattern = "*conf[._-]*xml"
    backup_pattern = "*conf[._-]*xml.20[0-9][0-9]-[01][0-9]-[0-3][0-9]_*"
    excludes = ["%gconf.xml"]

    for cpath, subdirs, files in walk(conf_dir):
        for name in files:
            if exclude_backups and fnmatch(name, backup_pattern):
                continue
            if fnmatch(name, pattern):
                filepath = path.join(cpath, name)
                entry = path.relpath(filepath, conf_dir)
                if not path.islink(filepath) and entry not in excludes:
                    conf_files.append(entry)

    conf_files.sort()
    return conf_files

def get_list_of_controlpanel_files(exclude_backups = 1):
    controlpanel_files = []
    pattern = "*control_panel[._-]*xml"
    backup_pattern = "*control_panel[._-]*xml.20[0-9][0-9]-[01][0-9]-[0-3][0-9]_*"
    excludes = []

    for cpath, subdirs, files in walk(conf_dir):
        for name in files:
            if exclude_backups and fnmatch(name, backup_pattern):
                continue
            if fnmatch(name, pattern):
                filepath = path.join(cpath, name)
                entry = path.relpath(filepath, conf_dir)
                if not path.islink(filepath) and entry not in excludes:
                    controlpanel_files.append(entry)
    controlpanel_files.sort()
    return controlpanel_files



def get_list_of_files(directory, extension):
    mylist = glob.glob(path.join(directory, "*" + extension))
    mylist.sort()
    ret = []
    for it in mylist:
        ret.append( it.replace(directory, "").replace(extension, ""))
    return ret

def get_list_of_modules():
    return get_list_of_files( modules_dir, ".xml")

def get_list_of_firmwares():
    return get_list_of_files( firmwares_dir, ".xml")

def get_list_of_boards():
    return get_list_of_files( boards_dir, ".xml")

def get_list_of_flight_plan_files():
    mylist = glob.glob(path.join(flight_plan_dir, "*.xml"))
    mylist.sort()
    return mylist

def get_list_of_servo_drivers():
    # \todo where do we know this?
    return ["Ppm", "Asctec", "Scilab"]

def get_module_information(module_name):
    str_desc = ""
    lst_def = []
    lst_conf = []
    try:
        xml = ET.parse(path.join(modules_dir, module_name + ".xml"))
        root = xml.getroot().find("doc")
        str_desc = root.find("description").text
        for block in root.iter("define"):
            lst_def.append([block.get("name"), block.get("value"), block.get("unit"), block.get("description")])
        for block in root.iter("configure"):
            lst_conf.append([block.get("name"), block.get("value"), block.get("unit"), block.get("description")])
    except (IOError, ET.XMLSyntaxError) as e:
        print(e.__str__())

    return PprzModule(description=str_desc, defines=lst_def, configures=lst_conf)


def search(string):
    #return call(["grep", "-r", string , PAPARAZZI_HOME + "/sw/airborne/"])
    #return system("grep -r " + string + " " + PAPARAZZI_HOME + "/sw/airborne/")
    cmd = "grep -r " + string + " " + PAPARAZZI_HOME + "/sw/airborne/"
    status, output = commands.getstatusoutput(cmd)
    return output.replace(PAPARAZZI_HOME + "/sw/airborne/", "")


if __name__ == '__main__':
    print("\nPAPARAZZI\n=========\n\nContent listing of current branch\n")
    print("\nBOARDS\n------\n")
    boards = get_list_of_boards()
    for b in boards:
      print(" - ```" + b + "```" )
    print("\nFIRMWARES\n---------\n")
    firmwares = get_list_of_firmwares()
    print("\nMODULES\n-------\n")
    modules  = get_list_of_modules()
    for m in modules:
        info = get_module_information(m)
        d = info.description
        if ((d is None) or (len(d) == 0)):
            d = " "
        print(" - ```" + m + "``` " + d.split('\n', 1)[0])
#    for mod in get_list_of_modules():
#        print(mod, " ---> ", get_module_information(mod))

