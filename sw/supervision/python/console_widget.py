# Copyright (C) 2008-2026 The Paparazzi Team
# released under GNU GPLv2 or later. See COPYING file.
from generated.ui_console import Ui_Console
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QProcess, QByteArray, Qt, QTimer, pyqtSignal
# QTextCursor likly no longer needed
from PyQt5.QtGui import QTextCursor
import utils
from program_widget import ProgramWidget
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional
import re
from conf import Aircraft


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

    def progress_stage(self):
        if self.emitter is None or not self.emitter.shortname.startswith("Flash "):
            return None
        match = re.match(r"^\s*(Erase|Program|Verify)\s*:", self.data, re.IGNORECASE)
        return match.group(1).lower() if match is not None else None

    def has_incomplete_progress(self):
        return self.progress_stage() is not None \
            and re.search(r"\b100(?:\.0+)?%", self.data) is None


class ConsoleWidget(QWidget, Ui_Console):

    # Future GUI hook: connect this to a main-window progress bar. The first
    # argument identifies the flash job; the second is flash_progress_value.
    flash_progress_changed = pyqtSignal(object, int)

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
        self.active_flash_programs = set()
        self.last_flash_progress = {}
        # Encoded flash progress: 0-100=Erase, 101-200=Program and
        # 201-300=Verify. Values 101/201 represent the 0-1% start of those
        # stages so every encoded value unambiguously identifies one stage.
        self.flash_progress_value = 0
        self.current_aircraft: Optional[Aircraft] = None
        self.programs_checkbox.stateChanged.connect(self.handle_check_all)
        self.log_level_slider.valueChanged.connect(self.log_level_changed)
        self.clear_button.clicked.connect(self.clear)
        self.splitter.setSizes([500, 100])
        self.progress_blink_visible = True
        self.progress_blink_timer = QTimer(self)
        self.progress_blink_timer.setInterval(500)
        self.progress_blink_timer.timeout.connect(self.toggle_progress_blink)
        self.progress_inactivity_timer = QTimer(self)
        self.progress_inactivity_timer.setSingleShot(True)
        self.progress_inactivity_timer.setInterval(2000)
        self.progress_inactivity_timer.timeout.connect(self.start_progress_blink)

    def set_aircraft(self, ac: Aircraft):
        self.current_aircraft = ac
        self.update_content()

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

        record_data = record.data
        if record.progress_stage() is not None:
            record_data = re.sub(
                r"\[([= ]*)\]",
                lambda match: "[" + match.group(1).replace(" ", "&nbsp;") + "]",
                record_data
            )
        if record.emitter in self.active_flash_programs and record.has_incomplete_progress():
            blink_character = "=" if self.progress_blink_visible \
                else '<span style="color:transparent;">=</span>'
            record_data = re.sub(r"(\[[=]*)&nbsp;", r"\1" + blink_character, record_data, count=1)
        data = "<span style=\"{}{}\">{}</span>".format(bg, ch, record_data)
        self.console_textedit.append(data)

    def toggle_progress_blink(self):
        self.progress_blink_visible = not self.progress_blink_visible
        scrollbar = self.console_textedit.verticalScrollBar()
        was_at_bottom = scrollbar.value() == scrollbar.maximum()
        scroll_position = scrollbar.value()
        self.update_content()
        scrollbar.setValue(scrollbar.maximum() if was_at_bottom else scroll_position)

    def has_incomplete_flash_progress(self):
        return any(
            r.emitter in self.active_flash_programs and r.has_incomplete_progress()
            for r in self.records
        )

    def start_progress_blink(self):
        if self.has_incomplete_flash_progress():
            self.toggle_progress_blink()
            self.progress_blink_timer.start()

    def reset_progress_blink_delay(self):
        self.progress_blink_timer.stop()
        self.progress_blink_visible = True
        if self.has_incomplete_flash_progress():
            self.progress_inactivity_timer.start()
        else:
            self.progress_inactivity_timer.stop()

    def update_progress_blink_timer(self):
        if not self.has_incomplete_flash_progress():
            self.progress_inactivity_timer.stop()
            self.progress_blink_timer.stop()
            self.progress_blink_visible = True

    def classify(self, line: str):
        for level, regs in self.LEVELS_REG.items():
            for reg in regs:
                if reg in line.lower():
                    return level
        return Level.ALL

    def update_flash_progress_value(self, record: Record):
        """Prepare encoded progress for a future main-window progress bar."""
        stage = record.progress_stage()
        percentage_match = re.search(r"(\d+(?:\.\d+)?)\s*%", record.data)
        if stage is None or percentage_match is None:
            return None

        raw_percentage = max(0.0, min(100.0, float(percentage_match.group(1))))
        percentage = int(raw_percentage + 0.5)
        stage_min, stage_max = {
            "erase": (0, 100),
            "program": (101, 200),
            "verify": (201, 300),
        }[stage]
        encoded_percentage = percentage if stage == "erase" \
            else stage_min + max(0, percentage - 1)
        encoded_percentage = max(stage_min, min(stage_max, encoded_percentage))
        self.flash_progress_value = encoded_percentage
        # Skeleton only: no GUI consumes this signal yet.
        self.flash_progress_changed.emit(record.emitter, self.flash_progress_value)
        return stage, raw_percentage

    def handle_data(self, pw: ProgramWidget, data: QByteArray, channel: Channel):
        if pw not in self.p_checkboxes:
            self.new_program(pw)
        is_flash = pw.shortname.startswith("Flash ")
        if is_flash:
            lines = re.split(r"[\r\n]+", bytes(data).decode(errors="replace"))
        else:
            if data.endsWith(b'\n'):
                data = data[:-1]
            lines = data.split(b'\n')
        content_changed = False
        progress_changed = False
        for line in lines:
            if not is_flash:
                line = line.data().decode()
            # remove VT100 escape codes
            while True:
                m = re.match(r".*(\x1b\[((?:\d+;)*\d+)([mhK])).*", line)
                if m is not None:
                    seq = m.group(1)
                    line = line.replace(seq, "")
                else:
                    break
            #remove empty lines
            if line == "":
                continue

            level = self.classify(line)
            r = Record(level, line, pw, channel)
            r.aircraft = getattr(pw, "aircraft", None)
            progress_stage = r.progress_stage()
            if progress_stage is not None:
                self.active_flash_programs.add(pw)
                progress = self.update_flash_progress_value(r)
                if progress is not None and self.last_flash_progress.get(pw) != progress:
                    self.last_flash_progress[pw] = progress
                    progress_changed = True
                for index in range(len(self.records) - 1, -1, -1):
                    previous = self.records[index]
                    if previous.emitter == pw and previous.channel == channel \
                            and previous.progress_stage() is not None:
                        self.records[index] = r
                        content_changed = True
                        break
                else:
                    self.records.append(r)
                    content_changed = True
                continue
            self.records.append(r)
            if self.filter(r):
                self.display_record(r)
        if content_changed:
            if progress_changed:
                self.reset_progress_blink_delay()
            self.update_content()

    def handle_stdout(self, pw: ProgramWidget):
        data = pw.process.readAllStandardOutput()
        self.handle_data(pw, data, Channel.STDOUT)

    def handle_stderr(self, pw: ProgramWidget):
        data = pw.process.readAllStandardError()
        self.handle_data(pw, data, Channel.STDERR)

    def handle_program_finished(self, pw: ProgramWidget, exit_code: int, exit_status: QProcess.ExitStatus):
        self.active_flash_programs.discard(pw)
        self.last_flash_progress.pop(pw, None)
        if exit_code == 0:
            if pw.shortname.startswith("Flash "):
                self.post_message(pw, "Done with flashing {}".format(pw.shortname[6:]), replace_progress=True)
            else:
                self.post_message(pw, "{} Done".format(pw.shortname))
        else:
            self.post_message(pw, "{} terminated with code {}".format(pw.shortname, exit_code))
            self.update_progress_blink_timer()

    def post_message(self, pw: ProgramWidget, msg, replace_progress=False):
        r = Record(Level.ALL, msg, pw, Channel.MANAGEMENT)
        r.aircraft = getattr(pw, "aircraft", None)
        if replace_progress:
            for index in range(len(self.records) - 1, -1, -1):
                previous = self.records[index]
                if previous.emitter == pw and previous.progress_stage() is not None:
                    self.records[index] = r
                    self.update_content()
                    self.update_progress_blink_timer()
                    return
        self.records.append(r)
        if self.filter(r):
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
        self.active_flash_programs.discard(pw)
        self.last_flash_progress.pop(pw, None)
        chk = self.p_checkboxes.pop(pw)
        if chk is not None:
            for r in self.records:
                if r.emitter == pw:
                    r.emitter = None
            self.programs_widget.layout().removeWidget(chk)
            chk.deleteLater()
            self.update_content()
            self.update_progress_blink_timer()

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
            if r.emitter not in self.p_checkboxes:
                return False
            if self.p_checkboxes[r.emitter].checkState() != Qt.Checked:
                return False
        if self.current_aircraft is not None:
            return getattr(r, "aircraft", getattr(r.emitter, "aircraft", None)) == self.current_aircraft
        return True

    def update_content(self):
        self.console_textedit.clear()
        for r in self.records:
            if self.filter(r):
                self.display_record(r)

    def clear(self):
        # TODO remove only filtered ? plus trashed ?
        self.records.clear()
        self.last_flash_progress.clear()
        self.update_content()
        self.update_progress_blink_timer()
