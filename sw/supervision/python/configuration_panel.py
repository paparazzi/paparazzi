from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QProcess
import utils
from generated.ui_configuration_panel import Ui_ConfigurationPanel
from program_widget import ProgramWidget, TabProgramsState
from conf import *
from programs_conf import Tool
import subprocess
import os


class ConfigurationPanel(QWidget, Ui_ConfigurationPanel):

    clear_error = QtCore.pyqtSignal()
    ac_edited = QtCore.pyqtSignal(Aircraft)
    config_file_changed = QtCore.pyqtSignal(str, str)
    program_state_changed = QtCore.pyqtSignal(TabProgramsState)

    def __init__(self, parent=None, *args, **kwargs):
        QWidget.__init__(self, parent=parent, *args, **kwargs)
        self.setupUi(self)
        self.console_widget.filter_widget.hide()
        self.currentAC = None   # type: Aircraft
        self.flight_plan_editor = None
        self.programs_state: TabProgramsState = TabProgramsState.IDLE
        self.conf_widget.file_changed.connect(self.handle_config_file_changed)
        self.conf_widget.conf_changed.connect(self.handle_conf_changed)
        self.conf_widget.setting_changed.connect(self.handle_setting_changed)
        self.conf_widget.flight_plan.edit_alt.connect(self.edit_flightplan_gcs)
        self.build_widget.spawn_program.connect(self.launch_program)

    def init(self):
        settings = utils.get_settings()
        window_size: QtCore.QSize = settings.value("ui/window_size", QtCore.QSize(1000, 600), QtCore.QSize)
        lpw = settings.value("ui/left_pane_width", 100, int)
        self.splitter.setSizes([lpw, window_size.width() - lpw])

    def set_ac(self, ac: Aircraft):
        if ac is None:
            self.conf_widget.setDisabled(True)
            self.currentAC = None
            self.conf_widget.reset()
            self.build_widget.update_targets(ac)
            return
        self.conf_widget.setDisabled(False)
        self.currentAC = ac
        self.conf_widget.set_ac(ac)
        self.build_widget.update_targets(ac)

    def display_config(self, config: AircraftConfig):
        self.currentAC = None
        self.conf_widget.setDisabled(False)
        self.conf_widget.set_config(config)

    def handle_setting_changed(self):
        self.apply_current_widget_config()

    def handle_conf_changed(self):
        self.apply_current_widget_config()

    def handle_config_file_changed(self, field: str, path: str):
        if self.currentAC is None:
            self.config_file_changed.emit(field, path)

    def apply_current_widget_config(self):
        if self.currentAC is None:
            return
        self.currentAC.set_config(self.conf_widget.get_config())
        self.ac_edited.emit(self.currentAC)
        # should we save each time a tiny change is made ? very inefficient !
        # self.conf.save()

    def handle_tools_changed(self, tools: Dict[str, Tool]):
        if "Flight Plan Editor" in tools:
            self.flight_plan_editor = tools["Flight Plan Editor"]

    def edit_flightplan_gcs(self, path):
        if self.flight_plan_editor is not None:
            if self.flight_plan_editor.command.startswith("$"):
                cmd = [self.flight_plan_editor.command[1:]]
            else:
                cmd = [os.path.join(utils.PAPARAZZI_SRC, self.flight_plan_editor.command)]
            for arg in self.flight_plan_editor.args:
                cmd += arg.args()

            cmd.append(os.path.join(utils.CONF_DIR, path))
            subprocess.Popen(cmd)
            # self.launch_program(self.flight_plan_editor.name, cmd, self.flight_plan_editor.icon)

    def launch_program(self, shortname, cmd, icon, cb, aircraft=None):
        pw = ProgramWidget(shortname, cmd, icon, self.programs_widget)
        pw.aircraft = aircraft if aircraft is not None else self.currentAC
        self.programs_widget.layout().addWidget(pw)
        pw.ready_read_stderr.connect(lambda: self.console_widget.handle_stderr(pw))
        pw.ready_read_stdout.connect(lambda: self.console_widget.handle_stdout(pw))
        pw.finished.connect(lambda c, s: self.handle_program_finished(pw, c, s))
        if cb is not None:
            pw.finished.connect(cb)
        pw.remove.connect(lambda: self.remove_program(pw))
        settings = utils.get_settings()
        if not settings.value("keep_build_programs", False, bool):
            pw.finished.connect(lambda: self.remove_program(pw))
        pw.start_program()
        self.programs_state = TabProgramsState.RUNNING
        self.program_state_changed.emit(self.programs_state)

    def handle_program_finished(self, pw: ProgramWidget, exit_code: int, exit_status: QProcess.ExitStatus):
        self.console_widget.handle_program_finished(pw, exit_code, exit_status)
        if exit_code != 0 and exit_code != 15:
            self.programs_state = TabProgramsState.ERROR
        else:
            if len(self.programs_widget.layout().children()) == 0 and self.programs_state != TabProgramsState.ERROR:
                self.programs_state = TabProgramsState.IDLE
        self.program_state_changed.emit(self.programs_state)

    def remove_program(self, pw: ProgramWidget):
        self.programs_widget.layout().removeWidget(pw)
        self.console_widget.remove_program(pw)
        pw.deleteLater()
