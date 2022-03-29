import os.path

from generated.ui_program import Ui_Program
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
from PyQt5.QtCore import QProcess
from PyQt5.QtGui import QIcon
import utils
from typing import List


class ProgramWidget(QWidget):

    ready_read_stdout = QtCore.pyqtSignal()
    ready_read_stderr = QtCore.pyqtSignal()
    finished = QtCore.pyqtSignal(int, QProcess.ExitStatus)
    remove = QtCore.pyqtSignal()

    def __init__(self, shortname: str, cmd: List[str], icon=None, parent=None):
        QWidget.__init__(self, parent=parent)
        self.ui = Ui_Program()
        self.ui.setupUi(self)
        self.cmd = cmd
        self.shortname = shortname
        self.process = QProcess(self)
        self.ui.program_label.setText(shortname)
        self.ui.program_lineedit.setText(" ".join(cmd))
        # self.ui.program_lineedit.hide()
        self.ui.program_label.hide()
        self.ui.shortview_checkbox.toggled.connect(self.toggle_view)
        self.ui.program_lineedit.returnPressed.connect(self.handle_cmd_return)
        self.ui.run_button.clicked.connect(self.handle_run)
        self.ui.remove_button.clicked.connect(self.handle_remove)
        self.process.readyReadStandardOutput.connect(self.ready_read_stdout)
        self.process.readyReadStandardError.connect(self.ready_read_stderr)
        self.process.finished.connect(self.handle_finished)
        self.process.started.connect(self.handle_started)
        self.process.errorOccurred.connect(self.handle_error)
        i = QIcon(os.path.join(utils.PAPARAZZI_HOME, "data", "pictures", "tools_icons", icon))
        self.ui.icon_label.setPixmap(i.pixmap(20, 20))

    def start_program(self):
        if self.process.state() == QProcess.NotRunning:
            self.process.start(self.cmd[0], self.cmd[1:])

    def toggle_view(self, long_view):
        self.ui.program_label.setVisible(long_view)
        self.ui.program_lineedit.setVisible(not long_view)

    def handle_cmd_return(self):
        if self.process.state() == QProcess.NotRunning:
            self.cmd = self.ui.program_lineedit.text().split(" ")
            self.start_program()

    def handle_run(self):
        if self.process.state() == QProcess.NotRunning:
            self.cmd = self.ui.program_lineedit.text().split(" ")
            self.start_program()
        elif self.process.state() == QProcess.Running:
            self.process.terminate()

    def handle_remove(self):
        if self.process.state() == QProcess.NotRunning:
            self.remove.emit()
        elif self.process.state() == QProcess.Running:
            self.process.finished.connect(self.remove)
            self.process.terminate()

    def handle_started(self):
        icon = QIcon.fromTheme("media-playback-stop")
        self.ui.run_button.setIcon(icon)
        self.ui.program_lineedit.setReadOnly(True)

    def handle_finished(self, exit_code: int, exit_status: QProcess.ExitStatus):
        icon = QIcon.fromTheme("media-playback-start")
        self.ui.run_button.setIcon(icon)
        self.ui.program_lineedit.setReadOnly(False)
        self.finished.emit(exit_code, exit_status)

    def handle_error(self, error: QProcess.ProcessError):
        print("error: ", error)

    def terminate(self):
        if self.process.state() != QProcess.NotRunning:
            self.process.terminate()

    def state(self):
        return self.process.state()
