
import os.path

import console_widget
from generated.ui_session import Ui_Session
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
import utils
import lxml.etree as ET

from typing import List, Optional, Tuple, Dict
from program_widget import ProgramWidget
from tools_menu import ToolMenu
from pprz_conf import *


class SessionWidget(QWidget):

    programs_all_stopped = QtCore.pyqtSignal()

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
        self.tools_menu = ToolMenu()
        self.tools_menu.tool_clicked.connect(self.handle_new_tool)
        self.init_tools_menu()
        sessions_names = [session.name for session in self.sessions]
        self.ui.sessions_combo.addItems(sessions_names)
        self.ui.start_session_button.clicked.connect(self.start_session)
        self.ui.startall_button.clicked.connect(self.start_all)
        self.ui.removeall_button.clicked.connect(self.remove_all)
        self.ui.stopall_button.clicked.connect(self.stop_all)
        self.ui.add_tool_button.clicked.connect(self.open_tools)

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
            cmd = [tool.command[1:]] + flat_args
        else:
            cmd = [os.path.join(utils.PAPARAZZI_SRC, tool.command)] + flat_args

        pw = ProgramWidget(tool.name, cmd, tool.icon, self.ui.programs_widget)
        self.program_widgets.append(pw)
        lay: QVBoxLayout = self.ui.programs_widget.layout()
        lay.insertWidget(lay.count()-1, pw)
        pw.ready_read_stderr.connect(lambda: self.console.handle_stderr(pw))
        pw.ready_read_stdout.connect(lambda: self.console.handle_stdout(pw))
        pw.finished.connect(lambda c, s: self.console.handle_program_finished(pw, c, s))
        pw.remove.connect(lambda: self.remove_program(pw))
        # if REMOVE_PROGRAMS_FINISHED:
        #     pw.finished.connect(lambda: self.remove_program(pw))
        pw.start_program()
        self.console.new_program(pw)

    def remove_program(self, pw: ProgramWidget):
        self.console.remove_program(pw)
        pw.setParent(None)
        # self.ui.programs_widget.layout().removeWidget(pw)
        self.program_widgets.remove(pw)
        if len(self.program_widgets) == 0:
            self.programs_all_stopped.emit()
        # pw.deleteLater()

    def stop_all(self):
        for pw in self.program_widgets:
            pw.terminate()

    def start_all(self):
        for pw in self.program_widgets:
            pw.start_program()

    def remove_all(self):
        for pw in list(self.program_widgets):
            print("remove {}".format(pw.shortname))
            pw.handle_remove()

    def init_tools_menu(self):
        for t in self.tools.values():
            self.tools_menu.add_tool(t)

    def handle_new_tool(self, name):
        p = Program.from_tool(self.tools[name])
        self.launch_program(p)

    def open_tools(self):
        if self.tools_menu.isVisible():
            self.tools_menu.close()
        else:
            bottomLeft = self.mapToGlobal(self.ui.add_tool_button.geometry().bottomLeft())
            self.tools_menu.move(bottomLeft)
            self.tools_menu.show()
            self.tools_menu.setFocus(QtCore.Qt.PopupFocusReason)
