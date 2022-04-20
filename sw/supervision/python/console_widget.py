# Copyright (C) 2008-2022 The Paparazzi Team
# released under GNU GPLv2 or later. See COPYING file.
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


class Channel(Enum):
    STDOUT = 0
    STDERR = 1
    MANAGEMENT = 2


@dataclass
class Record:
    level: Level
    data: str
    emitter: ProgramWidget
    channel: Channel


class ConsoleWidget(QWidget, Ui_Console):

    LEVELS_REG = {
        Level.ERROR: ["error:", "error ", "no such file", "undefined reference", "failure", "multiple definition"],
        Level.WARNING: ["warning", "no srtm data found"],
        Level.INFO: ["pragma message", "info:", "paparazzi version", "build aircraft"]
    }

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.setupUi(self)
        self.records: List[Record] = []
        self.p_checkboxes: Dict[ProgramWidget, QCheckBox] = {}
        self.programs_checkbox.stateChanged.connect(self.handle_check_all)
        self.log_level_slider.valueChanged.connect(self.log_level_changed)
        self.clear_button.clicked.connect(self.clear)
        self.splitter.setSizes([500, 100])

    def display_record(self, record):
        if record.level == Level.ERROR:
            bg = "background-color:red;"
        elif record.level == Level.WARNING:
            bg = "background-color:orange;"
        elif record.level == Level.INFO:
            bg = "background-color:lime;"
        else:
            bg = ""

        if record.channel == Channel.STDOUT:
            ch = ""
        elif record.channel == Channel.STDERR:
            ch = "font-style: italic;"
        else:
            ch = "font-weight: bold;"

        data = "<span style=\"{}{}\">{}</span>".format(bg, ch, record.data)
        self.console_textedit.append(data)

    def classify(self, line: str):
        for level, regs in self.LEVELS_REG.items():
            for reg in regs:
                if reg in line.lower():
                    return level
        return Level.ALL

    def handle_data(self, pw: ProgramWidget, data: QByteArray, channel: Channel):
        if pw not in self.p_checkboxes:
            self.new_program(pw)
        if data.endsWith(b'\n'):
            data = data[:-1]
        lines = data.split(b'\n')
        for line in lines:
            line = line.data().decode()
            level = self.classify(line)
            r = Record(level, line, pw, channel)
            self.records.append(r)
            self.display_record(r)

    def handle_stdout(self, pw: ProgramWidget):
        data = pw.process.readAllStandardOutput()
        self.handle_data(pw, data, Channel.STDOUT)

    def handle_stderr(self, pw: ProgramWidget):
        data = pw.process.readAllStandardError()
        self.handle_data(pw, data, Channel.STDERR)

    def handle_program_finished(self, pw: ProgramWidget, exit_code: int, exit_status: QProcess.ExitStatus):
        if exit_code == 0:
            self.post_message(pw, "{} Done".format(pw.shortname))
        else:
            self.post_message(pw, "{} terminated with code {}".format(pw.shortname, exit_code))

    def post_message(self, pw: ProgramWidget, msg):
        r = Record(Level.ALL, msg, pw, Channel.MANAGEMENT)
        self.records.append(r)
        self.display_record(r)

    def new_program(self, pw: ProgramWidget):
        chk = QCheckBox(pw.shortname, self.programs_widget)
        chk.setToolTip(" ".join(pw.cmd))
        self.p_checkboxes[pw] = chk
        index = self.programs_widget.layout().count() - 1
        lay: QVBoxLayout = self.programs_widget.layout()
        lay.insertWidget(index, chk)
        chk.stateChanged.connect(self.handle_program_checked)
        self.handle_program_checked()

    def remove_program(self, pw: ProgramWidget):
        chk = self.p_checkboxes.pop(pw)
        if chk is not None:
            for r in self.records:
                if r.emitter == pw:
                    r.emitter = None
            self.programs_widget.layout().removeWidget(chk)
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
                    self.programs_checkbox.blockSignals(True)
                    self.programs_checkbox.setCheckState(Qt.PartiallyChecked)
                    self.programs_checkbox.blockSignals(False)
                    break
            else:
                self.programs_checkbox.blockSignals(True)
                self.programs_checkbox.setCheckState(state)
                self.programs_checkbox.blockSignals(False)
        self.update_content()

    def log_level_changed(self, value):
        if value == Level.ERROR.value:
            self.log_level_label.setText("Errors")
        elif value == Level.WARNING.value:
            self.log_level_label.setText("Warnings")
        elif value == Level.INFO.value:
            self.log_level_label.setText("Info")
        elif value == Level.ALL.value:
            self.log_level_label.setText("All")
        self.update_content()

    def filter(self, r: Record):
        log_level = self.log_level_slider.value()
        if r.level.value > log_level:
            return False
        # if any program is checked, display only those that are checked
        if self.programs_checkbox.checkState() != Qt.Unchecked:
            if r.emitter is None:
                return False
            if self.p_checkboxes[r.emitter].checkState() != Qt.Checked:
                return False
        return True

    def update_content(self):
        self.console_textedit.clear()
        for r in self.records:
            if self.filter(r):
                self.display_record(r)

    def clear(self):
        # TODO remove only filtered ? plus trashed ?
        self.records.clear()
        self.update_content()
