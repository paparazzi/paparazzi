from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui, QtWidgets
import utils
from generated.ui_configuration_panel import Ui_ConfigurationPanel
from program_widget import ProgramWidget
from conf import *
from programs_conf import parse_tools
import subprocess


class ConfigurationPanel(QWidget, Ui_ConfigurationPanel):

    clear_error = QtCore.pyqtSignal()
    ac_edited = QtCore.pyqtSignal(Aircraft)

    def __init__(self, parent=None, *args, **kwargs):
        QWidget.__init__(self, parent=parent, *args, **kwargs)
        self.setupUi(self)
        self.console_widget.filter_widget.hide()
        self.currentAC = None   # type: Aircraft
        self.flight_plan_editor = None
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
        self.currentAC = ac
        self.conf_widget.set_ac(ac)
        self.build_widget.update_targets(ac)

    def handle_setting_changed(self):
        def make_setting(item: QListWidgetItem):
            name = item.text()
            state = True if item.checkState() == QtCore.Qt.Checked else False
            return Setting(name, state)

        modules, settings = [], []
        for i in range(self.conf_widget.settings.settings.count()):
            item = self.conf_widget.settings.settings.item(i)
            s = make_setting(item)
            if item.text().startswith("module"):
                modules.append(s)
            else:
                settings.append(s)

        self.currentAC.settings_modules = modules
        self.currentAC.settings = settings
        self.ac_edited.emit(self.currentAC)
        # should we save each time a tiny change is made ? very inefficient !
        # self.conf.save()

    def handle_conf_changed(self):
        self.currentAC.airframe = self.conf_widget.airframe.path
        self.currentAC.flight_plan = self.conf_widget.flight_plan.path
        self.currentAC.radio = self.conf_widget.radio.path
        self.currentAC.telemetry = self.conf_widget.telemetry.path
        self.ac_edited.emit(self.currentAC)

    def edit_flightplan_gcs(self, path):
        if self.flight_plan_editor is None:
            tools = parse_tools()
            if "Flight Plan Editor" in tools:
                self.flight_plan_editor = tools["Flight Plan Editor"]

        if self.flight_plan_editor is not None:
            cmd = [os.path.join(utils.PAPARAZZI_SRC, self.flight_plan_editor.command)]
            for arg in self.flight_plan_editor.args:
                cmd += arg.args()
            cmd.append(os.path.join(utils.CONF_DIR, path))
            subprocess.Popen(cmd)
            # self.launch_program(self.flight_plan_editor.name, cmd, self.flight_plan_editor.icon)

    def launch_program(self, shortname, cmd, icon, cb):
        pw = ProgramWidget(shortname, cmd, icon, self.programs_widget)
        self.programs_widget.layout().addWidget(pw)
        pw.ready_read_stderr.connect(lambda: self.console_widget.handle_stderr(pw))
        pw.ready_read_stdout.connect(lambda: self.console_widget.handle_stdout(pw))
        pw.finished.connect(lambda c, s: self.console_widget.handle_program_finished(pw, c, s))
        if cb is not None:
            pw.finished.connect(cb)
        pw.remove.connect(lambda: self.remove_program(pw))
        settings = utils.get_settings()
        if not settings.value("keep_build_programs", False, bool):
            pw.finished.connect(lambda: self.remove_program(pw))
        pw.start_program()

    def remove_program(self, pw: ProgramWidget):
        self.programs_widget.layout().removeWidget(pw)
        self.console_widget.remove_program(pw)
        pw.deleteLater()
