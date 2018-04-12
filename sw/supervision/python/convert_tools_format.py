#!/usr/bin/python3
"""
This script convert tools from control panel to individual xml file in conf/tools/
This is the new format for tools. Tools should be removed from the control panel file.
"""

import xml.etree.ElementTree as Et
import lib.environment as env
import os

NAME_REF = "name"
PROGRAM_REF = "program"
SECTION_REF = "section"
PROGRAM_TAG_REF = "/".join((SECTION_REF, PROGRAM_REF))


def parse_tools(tools_file, output_dir):
    tree = Et.parse(tools_file)
    tools_tags = tree.findall(PROGRAM_TAG_REF)

    for i, tool_tag in enumerate(tools_tags):
        tool_name = tool_tag.get(NAME_REF)
        filename = tool_name.lower()
        filename = filename.translate ({ord(c): "_" for c in " !@#$%^&*()[]{};:,./<>?\|`~-=_+"})
        el = Et.ElementTree(tool_tag)
        el.write("{}/{}.xml".format(output_dir, filename))


if __name__ == '__main__':
    cp = "{}/conf/control_panel.xml".format(env.PAPARAZZI_HOME)
    outDir = "{}/conf/tools".format(env.PAPARAZZI_HOME)
    os.makedirs(outDir, exist_ok=True)
    parse_tools(cp, outDir)
