from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QProcess
import utils
from generated.ui_log_widget import Ui_LogWidget
from conf import *
import re
import shutil

TMP_DIR = "/tmp/pprz"

ENVS = [
    "PAPARAZZI_HOME",
    "PAPARAZZI_SRC",
    "HOME"
]

DATE_TAGS = ["YY", "MM", "DD", "hh", "mm", "ss"]
OTHER_TAGS = ["AIRCRAFT", "AC_ID"]


class LogWidget(QWidget, Ui_LogWidget):

    def __init__(self, parent=None, *args, **kwargs):
        QWidget.__init__(self, parent=parent, *args, **kwargs)
        self.setupUi(self)
        self.process = QProcess(self)
        self.date = None
        self.basename = None
        self.conf: Conf = None
        self.substitutions = {}
        self.reset_output()
        self.input_button.clicked.connect(self.select_input)
        self.output_button.clicked.connect(self.select_output)
        self.reset_output_button.clicked.connect(self.reset_output)
        self.process_button.clicked.connect(self.start_processing)
        self.help_button.clicked.connect(self.show_help)
        self.process.finished.connect(self.handle_finished)
        self.process.readyReadStandardOutput.connect(self.read_stdout)
        self.process.readyReadStandardError.connect(self.read_stderr)

    def set_conf(self, conf):
        self.conf = conf

    def select_input(self):
        home = os.path.expandvars("$HOME")
        (path, _) = QFileDialog().getOpenFileName(self, "Select log file", home, "Log (*.LOG);; All (*.*)")
        self.input_edit.setText(path)

    def select_output(self):
        start_path = os.path.expandvars(self.output_edit.text())
        path = QFileDialog().getExistingDirectory(self, "Select log file", start_path, QFileDialog.ShowDirsOnly)
        for env in ENVS:
            res = os.getenv(env)
            if res in path:
                path_sub = path.replace(res, f"${env}")
                if os.path.expandvars(path_sub) == path:
                    # check that the substitution is valid
                    path = path_sub
        self.output_edit.setText(path)

    def reset_output(self):
        settings = utils.get_settings()
        output_path = settings.value("default_log_path", "$PAPARAZZI_HOME/var/logs/$YY-$MM/$DD/$AIRCRAFT", str)
        self.output_edit.setText(output_path)

    def start_processing(self):
        if (self.process.state() == QProcess.ProcessState.NotRunning and
                self.input_edit.text() != ""):
            program = os.path.expandvars("$PAPARAZZI_SRC/sw/logalizer/sd2log")
            input = self.input_edit.text()
            output = os.path.expandvars(self.output_edit.text())
            if not os.path.exists(TMP_DIR):
                os.makedirs(TMP_DIR)
            self.process.start(program, [input, TMP_DIR])
            self.status_label.setText("processing...")

    def handle_finished(self, exit_code: int, exit_status: QProcess.ExitStatus):
        if exit_status != QProcess.ExitStatus.NormalExit or exit_code != 0:
            self.status_label.setText(f"exited with code {exit_code}")
            return

        self.status_label.setText("Log processed, copying files...")
        ac_id = self.get_ac_id(f"{TMP_DIR}/{self.basename}.data")
        ac = self.conf[ac_id]
        self.substitutions["AC_ID"] = str(ac_id)
        self.substitutions["AIRCRAFT"] = ac.name
        out = os.path.expandvars(self.output_edit.text())
        for tag in DATE_TAGS + OTHER_TAGS:
            try:
                value = self.substitutions[tag]
                out = out.replace(f"${tag}", value)
            except KeyError:
                print(f"no tag {tag}")
                self.status_label.setText(f"Error: unknown tag {tag}!")
                return
        if not os.path.exists(out):
            os.makedirs(out)
        shutil.move(f"{TMP_DIR}/{self.basename}.data", f"{out}/{self.basename}.data")
        shutil.move(f"{TMP_DIR}/{self.basename}.log", f"{out}/{self.basename}.log")
        shutil.move(f"{TMP_DIR}/{self.basename}.tlm", f"{out}/{self.basename}.tlm")
        self.status_label.setText(f"Files extracted in {out}")

    def get_ac_id(self, filename) -> int:
        with open(filename, 'r') as data:
            ac_id = int(data.readline().split()[1])
            return ac_id

    def read_stdout(self):
        out = self.process.readAllStandardOutput()
        out = out.data().decode()

    def read_stderr(self):
        err = self.process.readAllStandardError()
        err = err.data().decode()
        for line in err.splitlines():
            m = re.match(r'(.*)\.data', line)
            if m is not None:
                self.basename = m.group(1)
                m = re.match(r'(\d{2})_(\d{2})_(\d{2})__(\d{2})_(\d{2})_(\d{2})', self.basename)
                if len(m.groups()) == 6:
                    for tag, value in zip(DATE_TAGS, m.groups()):
                        self.substitutions[tag] = value

    def show_help(self):
        QMessageBox.information(self, "logs extractor help",
                                "<h1>Extract easily your log files!</h1><br/>"
                                "You can use substitution variables in the output directory:<br/>"
                                "<b>$AIRCRAFT</b>: aircraft name<br/>"
                                "<b>$AC_ID</b>: aircraft id<br/>"
                                "<b>$YY</b>: year<br/>"
                                "<b>$MM</b>: month<br/>"
                                "<b>$DD</b>: day<br/>"
                                "<b>$hh</b>: hour<br/>"
                                "<b>$mm</b>: minutes<br/>"
                                "<b>$ss</b>: seconds<br/>")
