
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
from conf import *
from console_widget import ConsoleWidget


class SessionWidget(QWidget):

    programs_all_stopped = QtCore.pyqtSignal()
    program_spawned = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.ui = Ui_Session()
        self.ui.setupUi(self)
        self.program_widgets: List[ProgramWidget] = []
        self.console: ConsoleWidget = None
        self.ac: Aircraft = None
        self.ui.menu_button.addAction(self.ui.save_session_action)
        self.ui.menu_button.addAction(self.ui.save_as_action)
        self.ui.menu_button.addAction(self.ui.rename_session_action)
        self.ui.menu_button.addAction(self.ui.remove_session_action)
        self.sessions = self.parse_session()
        self.tools = self.parse_tools()
        self.tools_menu = ToolMenu()
        self.tools_menu.tool_clicked.connect(self.handle_new_tool)
        self.init_tools_menu()
        self.ui.sessions_combo.addItems(["Simulation", "Replay"])
        sessions_names = [session.name for session in self.sessions]
        self.ui.sessions_combo.addItems(sessions_names)
        self.ui.sessions_combo.insertSeparator(2)
        self.ui.start_session_button.clicked.connect(self.start_session)
        self.ui.startall_button.clicked.connect(self.start_all)
        self.ui.removeall_button.clicked.connect(self.remove_all)
        self.ui.stopall_button.clicked.connect(self.stop_all)
        self.ui.add_tool_button.clicked.connect(self.open_tools)
        self.ui.save_session_action.triggered.connect(self.handle_save)
        self.ui.save_as_action.triggered.connect(self.handle_save_as)
        self.ui.rename_session_action.triggered.connect(self.handle_rename)
        self.ui.remove_session_action.triggered.connect(self.remove_session)

    def set_console(self, console: console_widget.ConsoleWidget):
        self.console = console

    def set_aircraft(self, ac: Aircraft):
        self.ac = ac

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
        combo_text = self.ui.sessions_combo.currentText()
        if combo_text == "Simulation":
            self.start_simulation()
        elif combo_text == "Replay":
            self.start_replay()
        else:
            for session in self.sessions:
                if session.name == combo_text:
                    for program in session.programs:
                        self.launch_program(program)

    def start_replay(self):
        lfp = Program.from_tool(self.tools["Log File Player"])
        server = Program.from_tool(self.tools["Server"])
        server.args.append(Arg("-n", None))
        gcs = Program.from_tool(self.tools["GCS"])
        self.launch_program(lfp)
        self.launch_program(server)
        self.launch_program(gcs)

    def start_simulation(self):
        print(self.ac)
        if "nps" not in self.ac.boards and "sim" not in self.ac.boards:
            self.console.post_message(None, "No simulation target for {}.".format(self.ac.name))
            return

        elif "nps" in self.ac.boards and "sim" in self.ac.boards:
            simulator, ok = QInputDialog.getItem(self, "Simulator", "Please choose the simulator:",
                                         ["nps", "sim"], editable=False)
            if not ok:
                return
        elif "nps" in self.ac.boards:
            simulator = "nps"
        else:
            # simulator is "sim"
            simulator = "sim"

        if simulator == "nps":
            t = self.tools["Simulator"]
            simu = Program.from_tool(t)
            simu.args.append(Arg("-t", "nps"))
            self.launch_program(simu)
            datalink = Program.from_tool(self.tools["Data Link"])
            datalink.args = [Arg("-udp", None), Arg("-udp_broadcast", None)]
            self.launch_program(datalink)
        else:
            # simulator is "sim"
            sim = Program.from_tool(self.tools["Simulator"])
            sim.args.extend([Arg("-t", "sim"), Arg("--boot", None), Arg("--norc", None)])
            self.launch_program(sim)

        server = Program.from_tool(self.tools["Server"])
        server.args.append(Arg("-n", None))
        gcs = Program.from_tool(self.tools["GCS"])
        self.launch_program(server)
        self.launch_program(gcs)

    def launch_program(self, program: Program):
        if self.console is None:
            raise Exception("Console not set!")
        tool = self.tools[program.name]
        args = [arg.args(self.ac) for arg in program.args]
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
        pw.finished.connect(lambda c, s: self.handle_program_finished(pw, c, s))
        pw.remove.connect(lambda: self.remove_program(pw))
        # if REMOVE_PROGRAMS_FINISHED:
        #     pw.finished.connect(lambda: self.remove_program(pw))
        pw.start_program()
        self.console.new_program(pw)
        self.program_spawned.emit()

    def remove_program(self, pw: ProgramWidget):
        self.console.remove_program(pw)
        pw.setParent(None)
        # self.ui.programs_widget.layout().removeWidget(pw)
        self.program_widgets.remove(pw)
        if len(self.program_widgets) == 0:
            self.programs_all_stopped.emit()
        # pw.deleteLater()

    def any_program_running(self):
        return any([pw.state() == QtCore.QProcess.Running for pw in self.program_widgets])

    def handle_program_finished(self, pw, c, s):
        self.console.handle_program_finished(pw, c, s)
        if not self.any_program_running():
            self.programs_all_stopped.emit()

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

    def handle_save(self):
        session_name = self.ui.sessions_combo.currentText()
        programs = self.get_programs()
        session = Session(session_name, programs)
        self.replace_session(session)
        self.save_sessions()

    def handle_save_as(self):
        print("yo")
        session_name, ok = QInputDialog.getText(self, "Session name", "enter the session name:")
        if not ok:
            return
        for session in self.sessions:
            if session.name == session_name:
                QMessageBox.warning(self, "Error", "A session with this name already exits.\nTry again with a new name.")
                return
        programs = self.get_programs()
        session = Session(session_name, programs)
        self.sessions.append(session)
        self.save_sessions()

    def handle_rename(self):
        for session_orig in self.sessions:
            if session_orig.name == self.ui.sessions_combo.currentText():
                break
        else:
            print("session not found")
            return
        session_name, ok = QInputDialog.getText(self, "Session name", "enter the session name:")
        if not ok:
            return
        for session in self.sessions:
            if session.name == session_name:
                QMessageBox.warning(self, "Error", "A session with this name already exits.\nTry again with a new name.")
                return
        session_orig.name = session_name
        self.save_sessions()

    def remove_session(self):
        for session in self.sessions:
            if session.name == self.ui.sessions_combo.currentText():
                self.sessions.remove(session)
                i = self.ui.sessions_combo.currentIndex()
                self.ui.sessions_combo.removeItem(i)
                self.save_sessions()
                return
        print("session {} not found".format(self.ui.sessions_combo.currentText()))

    def replace_session(self, session):
        for i, s in enumerate(self.sessions):
            if s.name == session.name:
                self.sessions[i] = session
                break

    def save_sessions(self):
        ctrl_panel_path = os.path.join(utils.CONF_DIR, "control_panel.xml")
        parser = ET.XMLParser(remove_blank_text=True)
        control_panel = ET.parse(ctrl_panel_path, parser)
        xml_sessions = ET.Element("section")
        xml_sessions.set("name", "sessions")
        for session in self.sessions:
            xml_sessions.append(session.to_xml())
        for xml_section in control_panel.getroot().findall("section"):
            if xml_section.get("name") == "sessions":
                control_panel.getroot().replace(xml_section, xml_sessions)
                break
        control_panel.write(ctrl_panel_path, pretty_print=True)
        print("sessions saved saved to {}".format(ctrl_panel_path))

    def get_programs(self):
        programs = []
        for p in self.program_widgets:
            name = p.shortname
            args = []
            if len(p.cmd) > 0:
                arg = None
                for param in p.cmd[1:]:
                    if param.startswith("-"):
                        # if it start with "-", make a new arg
                        arg = Arg(param, None)
                        args.append(arg)
                    else:
                        if arg is not None and arg.flag.startswith("-"):
                            # if it don't starts with -, but the previous did, fill the constant
                            arg.constant = param
                        else:
                            # if it don't starts with -, nor the previous, its probably a mandatory argument
                            arg = Arg(param, None)
                            args.append(arg)
            program = Program(name, args)
            programs.append(program)
        return programs
