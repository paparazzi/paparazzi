#!/usr/bin/env python
# coding: utf-8

r"""Paparazzi information retrieval"""

from __future__ import print_function

import glob

from collections import namedtuple
from os import getenv
from os.path import normpath, join, dirname, abspath
# from subprocess import call
# import commands

import lxml.etree as ET

import sys
if sys.version_info[0] == 2:
    PY2 = True
else:
    PY2 = False

if PY2 is True:
    from commands import getstatusoutput
else:
    from subprocess import getstatusoutput

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
home_dir = getenv("PAPARAZZI_HOME", normpath(join(dirname(abspath(__file__)),
                                                  '../../../')))

# Directories
firmwares_dir = join(home_dir, "conf/firmwares/")
modules_dir = join(home_dir, "conf/modules/")
airframes_dir = join(home_dir, "conf/airframes/")
boards_dir = join(home_dir, "conf/boards/")

# Structures
PprzModule = namedtuple("PprzModule", "description defines configures")


# List Of Stuff
def get_list_of_files(directory, extension):
    r"""Get list of files in given directory with given extension

    Parameters
    ----------
    directory : str
        Path to directory
    extension : str
        File extension including the dot

    Returns
    -------
    list[str] : list of files

    """
    mylist = glob.glob(join(directory, "*" + extension))
    mylist.sort()
    ret = []
    for it in mylist:
        ret.append(it.replace(directory, "").replace(extension, ""))
    return ret


def get_list_of_modules():
    return get_list_of_files(modules_dir, ".xml")


def get_list_of_firmwares():
    return get_list_of_files(firmwares_dir, ".makefile")


def get_list_of_boards():
    return get_list_of_files(boards_dir, ".makefile")


def get_list_of_subsystems(firmware):
    r"""Get list of subsystems for a given firmware

    Parameters
    ----------
    firmware : str
        The firmware name

    """
    subsys_dir = join(firmwares_dir, "subsystems/" + firmware + "/")
    # \todo how about shared
    # subsys_dir = join( firmwares_dir, "subsystems/shared/" )
    return get_list_of_files(subsys_dir, ".makefile")


def get_list_of_servo_drivers():
    # \todo where do we know this?
    return ["Ppm", "Asctec", "Scilab"]


def get_module_information(module_name):
    r"""Get information about a module

    Parameters
    ----------
    module_name : str

    Returns
    -------
    namedtuple[str, list, list] : PprzModule

    """
    str_desc = ""
    lst_def = []
    lst_conf = []
    try:
        xml = ET.parse(join(modules_dir, module_name + ".xml"))
        root = xml.getroot().find("doc")
        str_desc = root.find("description").text
        for block in root.iter("define"):
            lst_def.append([block.get("name"),
                            block.get("value"),
                            block.get("unit"),
                            block.get("description")])
        for block in root.iter("configure"):
            lst_conf.append([block.get("name"),
                             block.get("value"),
                             block.get("unit"),
                             block.get("description")])
    except (IOError, ET.XMLSyntaxError) as e:
        print(e.__str__())

    return PprzModule(description=str_desc,
                      defines=lst_def,
                      configures=lst_conf)


def search(string):
    r"""Search for a string in the <homedir>/sw/airborne folder

    Parameters
    ----------
    string : str
        The string to search for

    Returns
    -------
    str

    """
    # return call(["grep", "-r", string , home_dir + "/sw/airborne/"])
    # return system("grep -r " + string + " " + home_dir + "/sw/airborne/")
    cmd = "grep -r " + string + " " + home_dir + "/sw/airborne/"
    status, output = getstatusoutput(cmd)
    return output.replace(home_dir + "/sw/airborne/", "")


if __name__ == '__main__':
    print("\nPAPARAZZI\n=========\n\nContent listing of current branch\n")
    print("\nBOARDS\n------\n")
    boards = get_list_of_boards()
    for b in boards:
        print(" - ```" + b + "```")
    print("\nFIRMWARES - SUBSYSTEMS\n---------\n")
    firmwares = get_list_of_firmwares()
    firmwares.append("shared")
    for f in firmwares:
        print(" - " + f)
        subsystems = get_list_of_subsystems(f)
        for s in subsystems:
            print("   - ```", s, "```")
    print("\nMODULES\n-------\n")
    modules = get_list_of_modules()
    for m in modules:
        info = get_module_information(m)
        d = info.description
        if (d is None) or (len(d) == 0):
            d = " "
        print(" - ```" + m + "``` " + d.split('\n', 1)[0])
#    for mod in get_list_of_modules():
#        print(mod, " ---> ", get_module_information(mod))
