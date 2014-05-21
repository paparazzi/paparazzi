#!/usr/bin/env python

from __future__ import print_function

import lxml.etree as ET
import StringIO

import xml_common
import paparazzi


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
    groupname = group_identification_string + search + group_identification_string
    target.append(ET.Comment(groupname))
    for block in source.getroot():
        if isinstance(block, ET._Comment):
            if block.__str__() == "<!--" + groupname + "-->":
                source.getroot().remove(block)


def reorganize_airframe_xml(airframe_xml):
    some_file_like_object = StringIO.StringIO("<airframe/>")
    airframe_xml_tree = ET.parse(some_file_like_object)
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
    find_and_add_sections_with_name(airframe_xml, airframe, "HORIZONTAL CONTROL")
    find_and_add_sections_with_name(airframe_xml, airframe, "VERTICAL CONTROL")
    find_and_add_sections_with_name(airframe_xml, airframe, "AGGRESSIVE")
    # Rotorcraft
    find_and_add_sections_with_name(airframe_xml, airframe, "STABILIZATION_RATE")
    find_and_add_sections_with_name(airframe_xml, airframe, "STABILIZATION_ATTITUDE")
    find_and_add_sections_with_name(airframe_xml, airframe, "GUIDANCE_V")
    find_and_add_sections_with_name(airframe_xml, airframe, "GUIDANCE_H")

    find_or_add_group(airframe_xml, airframe, "MISC")

    find_and_add(airframe_xml, airframe, "*")

    xml_common.indent(airframe)

    temp = airframe.findall("./*")
    for t in temp:
        t.tail = "\n\n  "

    #print(etree.tostring(airframe))
    #ET.ElementTree(airframe_xml_tree).write('test.xml')
    return airframe_xml_tree


def get_airframe_header(airframe_file):
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
    try:
        fullfile = ""
        with open(text_file) as inputFileHandle:
            fullfile = inputFileHandle.read()
            inputFileHandle.close()
        with open(text_file, 'w') as outputFileHandle:
            outputFileHandle.write(new_string)
            outputFileHandle.write(fullfile)
            outputFileHandle.close()
    except IOError:
        return


def load(airframe_file):
    try:
        my_xml = ET.parse(airframe_file)
        return [None, my_xml, get_airframe_header(airframe_file)]
    except (IOError, ET.XMLSyntaxError, ET.XMLSyntaxError) as e:
        print(" ERROR: Loading XML failed: ")
        print(e)
        quit()


def fill_tree_children(block, tree, parent):
    for elem in block:
        ename = elem.get("name")
        if ename is None:
            ename = ""
            # Only add sub-blocks if there are children
        if len(elem) or ((block.tag != "section") & (block.tag != "load")):
            if not isinstance(elem, ET._Comment):
                piter = tree.append(parent, [elem.tag.__str__() + " " + ename, elem])
                fill_tree_children(elem, tree, piter)


def fill_tree(my_xml, tree):
    root = my_xml.getroot()

    tree.clear()
    add_place = None
    for block in root:
        if not isinstance(block, ET._Comment):
            name = block.get("name")
            if name is None:
                name = ""

            # print(block.tag.__str__() + " " + name)
            piter = tree.append(add_place, [block.tag.__str__() + " " + name, block])
            fill_tree_children(block, tree, piter)
        else:
            add_place = tree.append(None, [block.__str__().replace("<!--" + group_identification_string, "[").replace(
                group_identification_string + "-->", "]"), block])


def defines(elem, grid):
    grid.clear()
    for e in elem.findall("./define"):
        grid.append(["define", e.get("name"), e.get("value"), e.get("unit"), e.get("description")])
    for e in elem.findall("./configure"):
        grid.append(["configure", e.get("name"), e.get("value"), e.get("unit"), e.get("description")])


if __name__ == '__main__':
    import sys

    outputfile = 'test.xml'
    if len(sys.argv) > 1:
        airframe_file = sys.argv[1]
        if len(sys.argv) > 2:
            outputfile = sys.argv[2]
            if len(sys.argv) > 3:
                outputfile = airframe_file
    else:
        airframe_file = "../../../conf/airframes/CDW/yapa_xsens.xml"

    print(airframe_file)
    [e, airframe, hdr] = load(airframe_file)
    xml = reorganize_airframe_xml(airframe)
    ET.ElementTree(xml.getroot()).write(outputfile)
    add_text_before_file(outputfile, hdr)
