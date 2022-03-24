import os.path

from generated.ui_console import Ui_Console
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QProcess
import utils
from program_widget import ProgramWidget



class ConsoleWidget(QWidget):

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.ui = Ui_Console()
        self.ui.setupUi(self)

    def handle_stdout(self, pw: ProgramWidget):
        data = pw.process.readAll()
        if data.endsWith(b'\n'):
            data = data[:-1]
        lines = data.split(b'\n')
        for line in lines:
            lower = line.toLower()
            if lower.contains(b"error"):
                line.prepend(b'<span style=\"background-color:red\">')
                line.append(b'</span>')
            if lower.contains(b"warning"):
                line.prepend(b'<span style=\"background-color:orange\">')
                line.append(b'</span>')
            if lower.contains(b"info"):
                line.prepend(b'<span style=\"background-color:green\">')
                line.append(b'</span>')
            self.ui.console_textedit.append(line.data().decode())

    def handle_stderr(self, pw: ProgramWidget):
        self.handle_stdout(pw)

    def handle_program_finished(self, pw: ProgramWidget, exit_code: int, exit_status: QProcess.ExitStatus):
        self.ui.console_textedit.append("Program terminated with code {}".format(exit_code))
