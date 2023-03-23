# Copyright (C) 2008-2022 The Paparazzi Team
# released under GNU GPLv2 or later. See COPYING file.
import os.path

import console_widget
from generated.ui_session import Ui_Session
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
import utils
import lxml.etree as ET

from typing import List, Optional, Tuple, Dict
from program_widget import ProgramWidget, TabProgramsState
from tools_menu import ToolMenu
from programs_conf import *
from conf import *
from console_widget import ConsoleWidget


class SessionWidget(QWidget, Ui_Session):

    programs_all_stopped = QtCore.pyqtSignal()
    program_spawned = QtCore.pyqtSignal()
    program_state_changed = QtCore.pyqtSignal(TabProgramsState)

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.setupUi(self)
        self.program_widgets: List[ProgramWidget] = []
        self.console: ConsoleWidget = None
        self.ac: Aircraft = None
        self.sessions = []
        self.tools = []
        self.tools_menu = ToolMenu()
        self.sessions_combo.addItems(["Simulation", "Replay"])
        self.sessions_combo.insertSeparator(2)
        self.programs_state: TabProgramsState = TabProgramsState.IDLE
        self.menu_button.addAction(self.save_session_action)
        self.menu_button.addAction(self.save_as_action)
        self.menu_button.addAction(self.rename_session_action)
        self.menu_button.addAction(self.remove_session_action)
        self.tools_menu.tool_clicked.connect(self.handle_new_tool)
        self.start_session_button.clicked.connect(self.start_session)
        self.startall_button.clicked.connect(self.start_all)
        self.removeall_button.clicked.connect(self.remove_all)
        self.stopall_button.clicked.connect(self.stop_all)
        self.add_tool_button.clicked.connect(self.open_tools)
        self.save_session_action.triggered.connect(self.handle_save)
        self.save_as_action.triggered.connect(self.handle_save_as)
        self.rename_session_action.triggered.connect(self.handle_rename)
        self.remove_session_action.triggered.connect(self.remove_session)

    def set_console(self, console: console_widget.ConsoleWidget):
        self.console = console

    def set_aircraft(self, ac: Aircraft):
        self.ac = ac

    def init(self):
        self.sessions = parse_sessions()
        self.tools = parse_tools()
        self.init_tools_menu()
        sessions_names = [session.name for session in self.sessions]
        self.sessions_combo.addItems(sessions_names)
        last_session = utils.get_settings().value("ui/last_session", None, str)
        if last_session is not None:
            self.sessions_combo.setCurrentText(last_session)

    def get_current_session(self) -> str:
        """
        :return: current session name in comboBox.
        """
        return self.sessions_combo.currentText()

    def start_session(self):
        self.reset_programs_status()
        combo_text = self.sessions_combo.currentText()
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
        gcs = Program.from_tool(self.tools["PprzGCS"])
        self.launch_program(lfp)
        self.launch_program(server)
        self.launch_program(gcs)

    def start_simulation(self):
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
        gcs = Program.from_tool(self.tools["PprzGCS"])
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

        pw = ProgramWidget(tool.name, cmd, tool.icon, self.programs_widget)
        self.program_widgets.append(pw)
        lay: QVBoxLayout = self.programs_widget.layout()
        lay.insertWidget(lay.count()-1, pw)
        pw.ready_read_stderr.connect(lambda: self.console.handle_stderr(pw))
        pw.ready_read_stdout.connect(lambda: self.console.handle_stdout(pw))
        pw.finished.connect(lambda c, s: self.handle_program_finished(pw, c, s))
        pw.started.connect(self.handle_program_started)
        pw.remove.connect(lambda: self.remove_program(pw))
        # if REMOVE_PROGRAMS_FINISHED:
        #     pw.finished.connect(lambda: self.remove_program(pw))
        pw.start_program()
        self.console.new_program(pw)
        self.program_spawned.emit()

    def remove_program(self, pw: ProgramWidget):
        self.console.remove_program(pw)
        pw.setParent(None)
        # self.programs_widget.layout().removeWidget(pw)
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

        if c != 0 and c != 15:
            self.programs_state = TabProgramsState.ERROR
        else:
            if not self.any_program_running() and self.programs_state != TabProgramsState.ERROR:
                self.programs_state = TabProgramsState.IDLE
        self.program_state_changed.emit(self.programs_state)

    def handle_program_started(self):
        self.programs_state = TabProgramsState.RUNNING
        self.program_state_changed.emit(self.programs_state)
    def reset_programs_status(self):
        self.programs_state = TabProgramsState.IDLE
        self.program_state_changed.emit(self.programs_state)

    def stop_all(self):
        for pw in self.program_widgets:
            pw.terminate()
        self.reset_programs_status()

    def start_all(self):
        for pw in self.program_widgets:
            pw.start_program()
        self.reset_programs_status()

    def remove_all(self):
        for pw in list(self.program_widgets):
            pw.handle_remove()
        self.reset_programs_status()

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
            bottomLeft = self.mapToGlobal(self.add_tool_button.geometry().bottomLeft())
            self.tools_menu.move(bottomLeft)
            self.tools_menu.show()
            self.tools_menu.setFocus(QtCore.Qt.PopupFocusReason)

    def handle_save(self):
        session_name = self.sessions_combo.currentText()
        programs = self.get_programs()
        session = Session(session_name, programs)
        self.replace_session(session)
        self.save_sessions()

    def handle_save_as(self):
        session_name, ok = QInputDialog.getText(self, "Session name", "enter the session name:")
        if not ok:
            return
        for session in self.sessions:
            if session.name == session_name:
                QMessageBox.warning(self, "Error", "A session with this name already exits.\nTry again with a new name.")
                return
        programs = self.get_programs()
        session = Session(session_name, programs)
        self.sessions_combo.addItem(session_name)
        self.sessions_combo.setCurrentText(session_name)
        self.sessions.append(session)
        self.save_sessions()

    def handle_rename(self):
        for session_orig in self.sessions:
            if session_orig.name == self.sessions_combo.currentText():
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
            if session.name == self.sessions_combo.currentText():
                self.sessions.remove(session)
                i = self.sessions_combo.currentIndex()
                self.sessions_combo.removeItem(i)
                self.save_sessions()
                return
        print("session {} not found".format(self.sessions_combo.currentText()))

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
        print("sessions saved to {}".format(ctrl_panel_path))

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
                        if arg is not None and arg.flag.startswith("-") and arg.constant is None:
                            # if it don't starts with -, but the previous did, and the constant is not yet filled, fill the constant
                            arg.constant = param
                        else:
                            # if it don't starts with -, nor the previous, its probably a mandatory argument
                            arg = Arg(param, None)
                            args.append(arg)
            program = Program(name, args)
            programs.append(program)
        return programs
