import os.path

from generated.ui_console import Ui_Console
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QProcess, QByteArray, Qt
from PyQt5.QtGui import QTextCursor
import utils
from program_widget import ProgramWidget
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List


class Level(Enum):
    ERROR = 0
    WARNING = 1
    INFO = 2
    ALL = 3


@dataclass
class Record:
    level: Level
    data: str
    emitter: object


class ConsoleWidget(QWidget):

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.ui = Ui_Console()
        self.ui.setupUi(self)
        self.records: List[Record] = []
        self.p_checkboxes: Dict[ProgramWidget, QCheckBox] = {}
        self.ui.programs_checkbox.stateChanged.connect(self.handle_check_all)
        self.ui.log_level_slider.valueChanged.connect(self.log_level_changed)
        self.ui.clear_button.clicked.connect(self.clear)

    def display_record(self, record):
        if record.level == Level.ERROR:
            data = "<span style=\"background-color:red\">{}</span>".format(record.data)
        elif record.level == Level.WARNING:
            data = "<span style=\"background-color:orange\">{}</span>".format(record.data)
        elif record.level == Level.INFO:
            data = "<span style=\"background-color:green\">{}</span>".format(record.data)
        else:
            data = record.data
        self.ui.console_textedit.append(data)

    def handle_data(self, pw: ProgramWidget, data: QByteArray):
        if pw not in self.p_checkboxes:
            self.new_program(pw)
        if data.endsWith(b'\n'):
            data = data[:-1]
        lines = data.split(b'\n')
        for line in lines:
            lower = line.toLower()
            level = Level.ALL
            if lower.contains(b"info"):
                level = Level.INFO
            if lower.contains(b"warning"):
                level = Level.WARNING
            if lower.contains(b"error"):
                level = Level.ERROR
            r = Record(level, line.data().decode(), pw)
            self.records.append(r)
            self.display_record(r)

    def handle_stdout(self, pw: ProgramWidget):
        data = pw.process.readAllStandardOutput()
        self.handle_data(pw, data)

    def handle_stderr(self, pw: ProgramWidget):
        data = pw.process.readAllStandardError()
        self.handle_data(pw, data)

    def handle_program_finished(self, pw: ProgramWidget, exit_code: int, exit_status: QProcess.ExitStatus):
        self.ui.console_textedit.append("Program terminated with code {}".format(exit_code))

    def new_program(self, pw: ProgramWidget):
        chk = QCheckBox(pw.shortname, self.ui.programs_widget)
        chk.setToolTip(" ".join(pw.cmd))
        self.p_checkboxes[pw] = chk
        index = self.ui.programs_widget.layout().count() - 1
        lay: QVBoxLayout = self.ui.programs_widget.layout()
        lay.insertWidget(index, chk)
        chk.stateChanged.connect(self.handle_program_checked)
        self.handle_program_checked()
        # self.ui.programs_groupbox.layout().addWidget(chk)

    def remove_program(self, pw):
        print("remove")
        chk = self.p_checkboxes.pop(pw)
        if chk is not None:
            for r in self.records:
                if r.emitter == pw:
                    r.emitter = None
            self.ui.programs_widget.layout().removeWidget(chk)
            # self.ui.programs_groupbox.layout().removeWidget(chk)
            chk.deleteLater()
            self.update_content()

    def handle_check_all(self, state):
        if state == Qt.PartiallyChecked:
            state = Qt.Checked
        for chk in self.p_checkboxes.values():
            chk.blockSignals(True)
            chk.setCheckState(state)
            chk.blockSignals(False)
        self.update_content()

    def handle_program_checked(self):
        chks = list(self.p_checkboxes.values())
        if len(chks) > 0:
            state = chks[0].checkState()
            for s in chks[1:]:
                if s.checkState() != state:
                    self.ui.programs_checkbox.blockSignals(True)
                    self.ui.programs_checkbox.setCheckState(Qt.PartiallyChecked)
                    self.ui.programs_checkbox.blockSignals(False)
                    break
            else:
                self.ui.programs_checkbox.blockSignals(True)
                self.ui.programs_checkbox.setCheckState(state)
                self.ui.programs_checkbox.blockSignals(False)
        self.update_content()

    def log_level_changed(self, value):
        if value == Level.ERROR.value:
            self.ui.log_level_label.setText("Errors")
        elif value == Level.WARNING.value:
            self.ui.log_level_label.setText("Warnings")
        elif value == Level.INFO.value:
            self.ui.log_level_label.setText("Info")
        elif value == Level.ALL.value:
            self.ui.log_level_label.setText("All")
        self.update_content()

    def filter(self, r: Record):
        log_level = self.ui.log_level_slider.value()
        if r.level.value > log_level:
            return False
        # if any program is checked, display only those that are checked
        if self.ui.programs_checkbox.checkState() != Qt.Unchecked:
            if r.emitter is None:
                return False
            if self.p_checkboxes[r.emitter].checkState() != Qt.Checked:
                return False
        return True

    def update_content(self):
        self.ui.console_textedit.clear()
        for r in self.records:
            if self.filter(r):
                self.display_record(r)

    def clear(self):
        # TODO remove only filtered ? plus trashed ?
        self.records.clear()
        self.update_content()
