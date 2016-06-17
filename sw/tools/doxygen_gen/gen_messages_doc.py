#!/usr/bin/env python
from __future__ import print_function

from optparse import OptionParser
import xml.etree.ElementTree as ET

import os
import sys
import string
import textwrap


def print_field(field):
    name = field.get('name')
    type = field.get('type')
    desc = ''
    if field.text is not None:
        desc = field.text.strip()
    unit = field.get('unit', '')
    # if values are given, put those in the unit column
    if not unit:
        values = field.get('values', '')
        unit = string.replace(values, '|', ', ')
    return u"{name} | {type} | {unit} | {desc} |\n".format(name=name, type=type, desc=desc, unit=unit)


def print_message(message):
    label = "{#" + message.get('name') + "}"
    s = "## {0} (ID {1}) ## {2}\n\n".format(message.get('name'), message.get('id'), label)
    desc = message.find('description')
    if desc is not None:
        s += desc.text.strip() + "\n\n"
    fields = message.findall('./field')
    if fields:
        s += "Field name | Type | Unit/Values | Description |\n"
        s += "-----------|------|-------------|-------------|\n"
    else:
        s += "message has no fields\n"
    for f in fields:
        s += print_field(f)
    s += "\n\n"
    return s


def print_msg_class(msg_class):
    cls_name = msg_class.get('name').title()
    label = " {#" + msg_class.get('name').upper() + "_CLASS}"
    s = "# {0} Messages # {1}\n\n".format(cls_name, label)
    for m in msg_class.findall("./message"):
        s +=\
            print_message(m)
    return s


def messages_page(messages):
    page_header = """\
    Paparazzi Messages {#paparazzi_messages}
    ==================

    These are the common messages.
    Also see http://wiki.paparazziuav.org/wiki/Telemetry and http://wiki.paparazziuav.org/wiki/Messages_Format

    [TOC]

    """
    s = textwrap.dedent(page_header)

    for msg_class in messages.findall("./msg_class"):
        s += print_msg_class(msg_class)
        s += "\n"

    return s


def read_messages_file(file):
    try:
        tree = ET.parse(file)
    except ET.ParseError:
        print("Error. Xml file {0} is not well formed.".format(file))
        return None

    root = tree.getroot()
    if root.tag != "protocol":
        print("Error. Xml file {0} doesn't have 'protocol' as root node.".format(file))
        return None
    else:
        return root


if __name__ == '__main__':
    usage = "Usage: %prog [options]" + "\n" + "Run %prog --help to list the options."
    parser = OptionParser(usage)
    parser.add_option("-f", "--file", dest="file",
                      help="messages file to read [default: PAPARAZZI_HOME/sw/ext/pprzlink/message_definitions/v1.0/messages.xml]", metavar="MESSAGES.XML")
    parser.add_option("-o", "--outputdir", dest="output_dir",
                      help="write output to DIR [default: PAPARAZZI_HOME/doc/manual/generated]", metavar="DIR")
    parser.add_option("-p", "--parents",
                      action="store_true", dest="create_parent_dirs",
                      help="Create parent dirs of output dir if they don't exist.")
    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose")
    (options, args) = parser.parse_args()

    # if PAPARAZZI_HOME not set, then assume the tree containing this
    # file is a reasonable substitute
    paparazzi_home = os.getenv(
        "PAPARAZZI_HOME",
        os.path.normpath(os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '../../../')))

    if options.file:
        messages_file = options.file
    else:
        messages_file = os.path.join(paparazzi_home, "sw/ext/pprzlink/message_definitions/v1.0/messages.xml")
    if not os.path.isfile(messages_file):
        print("Messages file " + messages_file + " not found.")
        sys.exit(1)

    if options.output_dir:
        output_dir = options.output_dir
    else:
        output_dir = os.path.join(paparazzi_home, "doc/manual/generated")

    if not os.path.isdir(output_dir):
        if options.create_parent_dirs:
            print("Output directory " + output_dir + " doesn't exit yet. Creating it.")
            os.makedirs(output_dir)
        else:
            print("Output directory " + output_dir + " not valid.")
            sys.exit(1)

    if options.verbose:
        print("Generating messages documentation in " + output_dir)

    messages = read_messages_file(messages_file)

    # generate string containing markdown page
    outstring = messages_page(messages).encode('utf-8')

    #print(outstring)

    outfile_name = os.path.join(output_dir, "pprz_messages.md")
    with open(outfile_name, 'w') as outfile:
        outfile.write(outstring)
    if options.verbose:
        print("Done.")
