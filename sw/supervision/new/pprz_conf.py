from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict


@dataclass
class Arg:
    flag: str
    constant: Optional[str]

    @staticmethod
    def parse(xml_arg):
        flag = xml_arg.get("flag")
        constant = xml_arg.get("constant")
        return Arg(flag, constant)

    def args(self) -> List[str]:
        if self.constant is not None:
            return [self.flag, self.constant]
        else:
            return [self.flag]


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
        return Program(t.name, t.args)


@dataclass
class Session:
    name: str
    programs: List[Program] = field(default_factory=list)

    @staticmethod
    def parse(xml_session):
        name = xml_session.get("name")
        programs = [Program.parse(xml_program) for xml_program in xml_session.findall("program")]
        return Session(name, programs)


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
