# Copyright (C) 2008-2022 The Paparazzi Team
# released under GNU GPLv2 or later. See COPYING file.
from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict
from lxml import etree as ET
from conf import *
from copy import deepcopy


@dataclass
class Arg:
    flag: str
    constant: Optional[str]

    @staticmethod
    def parse(xml_arg):
        flag = xml_arg.get("flag")
        constant = xml_arg.get("constant")
        return Arg(flag, constant)

    def args(self, ac: Aircraft = None) -> List[str]:
        if self.constant is not None:
            constant = self.constant
            if "@AIRCRAFT" in self.constant:
                constant = self.constant.replace("@AIRCRAFT", ac.name)
            if "@AC_ID" in self.constant:
                constant = constant.replace("@AC_ID", str(ac.ac_id))
            return [self.flag, constant]
        else:
            return [self.flag]

    def to_xml(self) -> ET.Element:
        xml = ET.Element("arg")
        xml.set("flag", self.flag)
        if self.constant is not None:
            xml.set("constant", self.constant)
        return xml


@dataclass
class Program:
    name: str
    args: List[Arg] = field(default_factory=list)

    @staticmethod
    def parse(xml_program):
        name = xml_program.get("name")
        args = [Arg.parse(xml_arg) for xml_arg in xml_program.findall("arg")]
        return Program(name, args)

    @staticmethod
    def from_tool(t: Tool):
        return Program(t.name, deepcopy(t.args))

    def to_xml(self) -> ET.Element:
        xml: ET._Element = ET.Element("program")
        xml.set("name", self.name)
        for arg in self.args:
            xml.append(arg.to_xml())
        return xml


@dataclass
class Session:
    name: str
    programs: List[Program] = field(default_factory=list)

    @staticmethod
    def parse(xml_session):
        name = xml_session.get("name")
        programs = [Program.parse(xml_program) for xml_program in xml_session.findall("program")]
        return Session(name, programs)

    def to_xml(self) -> ET.Element:
        xml = ET.Element("session")
        xml.set("name", self.name)
        for p in self.programs:
            xml.append(p.to_xml())
        return xml


@dataclass
class Tool:
    name: str
    command: str
    icon: Optional[str]
    args: List[Arg]
    favorite: bool

    @staticmethod
    def parse(xml_program) -> Tuple[str, Tool]:
        name = xml_program.get("name")
        command = xml_program.get("command")
        icon = xml_program.get("icon")
        if icon is None:
            icon = "default_tool_icon.svg"
        favorite = True if xml_program.get("favorite") is not None else False
        args = [Arg.parse(xml_arg) for xml_arg in xml_program.findall("arg")]
        return name, Tool(name, command, icon, args, favorite)


def parse_tools() -> Dict[str, Tool]:
    tools = {}
    tools_dir = os.path.join(utils.CONF_DIR, "tools")
    for file in os.listdir(tools_dir):
        if file.endswith(".xml"):
            path = os.path.join(utils.CONF_DIR, "tools", file)
            xml = ET.parse(path).getroot()
            if xml.tag == "program":
                name, tool = Tool.parse(xml)
                tools[name] = tool
            else:
                print("unexpected tag ", xml.tag)

    # programs from control_panel.xml
    # override programs from conf/tools/*.xml
    control_panel = ET.parse(os.path.join(utils.CONF_DIR, "control_panel.xml"))
    for xml_section in control_panel.getroot().findall("section"):
        if xml_section.get("name") == "programs":
            for xml_program in xml_section.findall("program"):
                name, tool = Tool.parse(xml_program)
                tools[name] = tool

    return tools


def parse_sessions() -> List[Session]:
    control_panel = ET.parse(os.path.join(utils.CONF_DIR, "control_panel.xml"))
    for xml_section in control_panel.getroot().findall("section"):
        if xml_section.get("name") == "sessions":
            return [Session.parse(xml_session) for xml_session in xml_section.findall("session")]
