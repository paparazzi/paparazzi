#!/usr/bin/env python
# coding: utf-8

r"""Airframe XML description management"""

from __future__ import print_function

import lxml.etree as etree

import xml_common
# import paparazzi

import sys
if sys.version_info[0] == 2:
    PY2 = True
else:
    PY2 = False

if PY2 is True:
    from StringIO import StringIO
else:
    from io import StringIO


def find_and_add(source, target, search):
    temp = source.getroot().findall("./" + search)
    for t in temp:
        xml_common.indent(t, 1)
    target.extend(temp)


def find_and_add_sections_with_name(source, target, find_name):
    temp = source.getroot().findall("./*[@name='" + find_name + "']")
    for t in temp:
        xml_common.indent(t, 1)
    target.extend(temp)


group_identification_string = " ************************* "


def find_or_add_group(source, target, search):
    groupname = "".join([group_identification_string,
                         search,
                         group_identification_string])
    target.append(etree.Comment(groupname))
    for block in source.getroot():
        if isinstance(block, etree._Comment):
            if block.__str__() == "<!--" + groupname + "-->":
                source.getroot().remove(block)


def reorganize_airframe_xml(airframe_xml):
    r"""Reorganize the airframe xml"""
    some_file_like_object = StringIO("<airframe/>")
    airframe_xml_tree = etree.parse(some_file_like_object)
    airframe = airframe_xml_tree.getroot()

    if 'name' not in airframe_xml.getroot().attrib:
        print("Airframe has no name!")
    else:
        airframe.set('name', airframe_xml.getroot().get('name'))

    find_or_add_group(airframe_xml, airframe, "FIRMWARE")
    find_and_add(airframe_xml, airframe, "firmware")
    find_and_add_sections_with_name(airframe_xml, airframe, "AUTOPILOT")

    find_or_add_group(airframe_xml, airframe, "MODULES")
    find_and_add(airframe_xml, airframe, "modules")

    find_or_add_group(airframe_xml, airframe, "ACTUATORS")
    find_and_add_sections_with_name(airframe_xml, airframe, "ACTUATORS_MKK")
    find_and_add_sections_with_name(airframe_xml, airframe, "ACTUATORS_MKK_V2")
    find_and_add(airframe_xml, airframe, "servos")
    find_and_add(airframe_xml, airframe, "commands")
    find_and_add(airframe_xml, airframe, "ap_only_commands")
    find_and_add(airframe_xml, airframe, "rc_commands")
    find_and_add_sections_with_name(airframe_xml, airframe, "AUTO1")
    find_and_add_sections_with_name(airframe_xml, airframe, "SERVO_MIXER_GAINS")
    find_and_add_sections_with_name(airframe_xml, airframe, "MIXER")
    find_and_add_sections_with_name(airframe_xml, airframe, "MIXING")
    find_and_add(airframe_xml, airframe, "command_laws")
    find_and_add_sections_with_name(airframe_xml, airframe, "TRIM")
    find_and_add_sections_with_name(airframe_xml, airframe, "FAILSAFE")

    find_or_add_group(airframe_xml, airframe, "SENSORS")
    find_and_add_sections_with_name(airframe_xml, airframe, "ADC")
    find_and_add_sections_with_name(airframe_xml, airframe, "INFRARED")
    find_and_add_sections_with_name(airframe_xml, airframe, "IMU")
    find_and_add_sections_with_name(airframe_xml, airframe, "AHRS")
    find_and_add_sections_with_name(airframe_xml, airframe, "INS")
    find_and_add_sections_with_name(airframe_xml, airframe, "XSENS")

    find_or_add_group(airframe_xml, airframe, "GAINS")

    # Fixedwing
    find_and_add_sections_with_name(airframe_xml, airframe,
                                    "HORIZONTAL CONTROL")
    find_and_add_sections_with_name(airframe_xml, airframe, "VERTICAL CONTROL")
    find_and_add_sections_with_name(airframe_xml, airframe, "AGGRESSIVE")

    # Rotorcraft
    find_and_add_sections_with_name(airframe_xml, airframe,
                                    "STABILIZATION_RATE")
    find_and_add_sections_with_name(airframe_xml, airframe,
                                    "STABILIZATION_ATTITUDE")
    find_and_add_sections_with_name(airframe_xml, airframe, "GUIDANCE_V")
    find_and_add_sections_with_name(airframe_xml, airframe, "GUIDANCE_H")

    find_or_add_group(airframe_xml, airframe, "MISC")

    find_and_add(airframe_xml, airframe, "*")

    xml_common.indent(airframe)

    temp = airframe.findall("./*")
    for t in temp:
        t.tail = "\n\n  "

    # print(etree.tostring(airframe))
    # ET.ElementTree(airframe_xml_tree).write('test.xml')
    return airframe_xml_tree


def get_airframe_header(airframe_file):
    r"""Airframe header

    Parameters
    ----------
    airframe_file : str
        Path to the airframe XML file

    Returns
    -------
    str

    """
    try:
        with open(airframe_file) as inputFileHandle:
            fullfile = inputFileHandle.read()
            pos = fullfile.find("<airframe")
            if pos > 0:
                return fullfile[0:pos]
            else:
                return ""
    except IOError:
        return ""


def add_text_before_file(text_file, new_string):
    r"""Prepend a file with some text

    Parameters
    ----------
    text_file : str
        Path to the file
    new_string : str
        The string to add at the beginning of the file

    """
    try:
        # fullfile = ""
        # Read
        with open(text_file) as inputFileHandle:
            fullfile = inputFileHandle.read()
            inputFileHandle.close()

        # Write
        with open(text_file, 'w') as outputFileHandle:
            outputFileHandle.write(new_string)
            outputFileHandle.write(fullfile)
            outputFileHandle.close()

    except IOError:
        return


def load(airframe_file):
    r"""Load and parse airframe XML file

    Parameters
    ----------
    airframe_file : str
        Path to the airframe file

    Returns
    -------
    list

    """
    try:
        my_xml = etree.parse(airframe_file)
        return [None, my_xml, get_airframe_header(airframe_file)]
    except (IOError, etree.XMLSyntaxError, etree.XMLSyntaxError) as e:
        print(" ERROR: Loading XML failed: ")
        print(e)
        quit()


def fill_tree_children(block, tree, parent):
    r"""Place an XML block under specified parent"""
    for elem in block:
        ename = elem.get("name")
        if ename is None:
            ename = ""
            # Only add sub-blocks if there are children
        if len(elem) or ((block.tag != "section") & (block.tag != "load")):
            if not isinstance(elem, etree._Comment):
                piter = tree.append(parent, [elem.tag.__str__() + " " + ename,
                                             elem])
                fill_tree_children(elem, tree, piter)


def fill_tree(my_xml, tree):
    r"""Replace XML in tree with my_xml"""
    root = my_xml.getroot()

    tree.clear()
    add_place = None
    for block in root:
        if not isinstance(block, etree._Comment):
            name = block.get("name")
            if name is None:
                name = ""

            # print(block.tag.__str__() + " " + name)
            piter = tree.append(add_place,
                                [block.tag.__str__() + " " + name, block])
            fill_tree_children(block, tree, piter)
        else:
            block_str = block.__str__().replace("<!--" + group_identification_string,
                                                "[")
            block_str = block_str.replace(group_identification_string + "-->",
                                          "]")
            add_place = tree.append(None, [block_str, block])


def defines(elem, grid):
    grid.clear()
    for e in elem.findall("./define"):
        grid.append(["define",
                     e.get("name"),
                     e.get("value"),
                     e.get("unit"),
                     e.get("description")])

    for e in elem.findall("./configure"):
        grid.append(["configure",
                     e.get("name"),
                     e.get("value"),
                     e.get("unit"),
                     e.get("description")])


if __name__ == '__main__':
    import sys

    outputfile = 'test.xml'
    if len(sys.argv) > 1:
        airframe_file_ = sys.argv[1]
        if len(sys.argv) > 2:
            outputfile = sys.argv[2]
            if len(sys.argv) > 3:
                outputfile = airframe_file_
    else:
        airframe_file_ = "../../../conf/airframes/examples/microjet.xml"

    print(airframe_file_)
    [e_, airframe_, hdr] = load(airframe_file_)
    xml = reorganize_airframe_xml(airframe_)
    etree.ElementTree(xml.getroot()).write(outputfile)
    add_text_before_file(outputfile, hdr)
