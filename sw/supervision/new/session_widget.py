from __future__ import annotations
import os.path

import console_widget
from generated.ui_session import Ui_Session
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
import utils
import lxml.etree as ET
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict
from program_widget import ProgramWidget

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
        favorite = True if xml_program.get("favorite") is not None else False
        args = [Arg.parse(xml_arg) for xml_arg in xml_program.findall("arg")]
        return name, Tool(name, command, icon, args, favorite)


class SessionWidget(QWidget):

    spawn_program = QtCore.pyqtSignal(str, list)

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.ui = Ui_Session()
        self.ui.setupUi(self)
        self.program_widgets: List[ProgramWidget] = []
        self.console = None
        self.ui.menu_button.addAction(self.ui.save_session_action)
        self.ui.menu_button.addAction(self.ui.duplicate_session_action)
        self.ui.menu_button.addAction(self.ui.new_session_action)
        self.sessions = self.parse_session()
        self.tools = self.parse_tools()
        sessions_names = [session.name for session in self.sessions]
        self.ui.sessions_combo.addItems(sessions_names)
        self.ui.start_session_button.clicked.connect(self.start_session)
        self.ui.stopall_button.clicked.connect(self.stop_all)

    def set_console(self, console: console_widget.ConsoleWidget):
        self.console = console

    @staticmethod
    def parse_session() -> List[Session]:
        control_panel = ET.parse(os.path.join(utils.CONF_DIR, "control_panel.xml"))
        for xml_section in control_panel.getroot().findall("section"):
            if xml_section.get("name") == "sessions":
                return [Session.parse(xml_session) for xml_session in xml_section.findall("session")]

    @staticmethod
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

    def start_session(self):
        for session in self.sessions:
            if session.name == self.ui.sessions_combo.currentText():
                for program in session.programs:
                    self.launch_program(program)

    def launch_program(self, program: Program):
        if self.console is None:
            raise Exception("Console not set!")
        tool = self.tools[program.name]
        args = [arg.args() for arg in program.args]
        flat_args = [item for sublist in args for item in sublist]
        if tool.command.startswith("$"):
            cmd = [tool.command] + flat_args
        else:
            cmd = [os.path.join(utils.PAPARAZZI_SRC, tool.command)] + flat_args
        self.spawn_program.emit(tool.name, cmd)
        print(tool.name, cmd)

        pw = ProgramWidget(tool.name, cmd, self.ui.programs_widget)
        self.program_widgets.append(pw)
        lay: QVBoxLayout = self.ui.programs_widget.layout()
        lay.insertWidget(lay.count()-1, pw)
        pw.ready_read_stderr.connect(lambda: self.console.handle_stderr(pw))
        pw.ready_read_stdout.connect(lambda: self.console.handle_stdout(pw))
        pw.finished.connect(lambda c, s: self.console.handle_program_finished(pw, c, s))
        pw.remove.connect(lambda: self.remove_program(pw))
        pw.destroyed.connect(lambda : self.console.remove_program(pw))
        # if REMOVE_PROGRAMS_FINISHED:
        #     pw.finished.connect(lambda: self.remove_program(pw))
        pw.start_program()

    def remove_program(self, pw: ProgramWidget):
        pw.disconnect()
        print("ye")
        self.ui.programs_widget.layout().removeWidget(pw)
        self.program_widgets.remove(pw)
        pw.deleteLater()

    def stop_all(self):
        for pw in self.program_widgets:
            pw.terminate()
