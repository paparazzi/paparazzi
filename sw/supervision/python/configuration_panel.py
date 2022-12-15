import os, sys, copy
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui, QtWidgets
import utils
from generated.ui_configuration_panel import Ui_ConfigurationPanel
from generated.ui_new_ac_dialog import Ui_Dialog
from program_widget import ProgramWidget
from conf import *
from programs_conf import parse_tools
import subprocess

PAPARAZZI_SRC = os.getenv("PAPARAZZI_SRC")
PAPARAZZI_HOME = os.getenv("PAPARAZZI_HOME", PAPARAZZI_SRC)
lib_path = os.path.normpath(os.path.join(PAPARAZZI_SRC, 'sw', 'lib', 'python'))
sys.path.append(lib_path)
import paparazzi

# TODO make a setting ?
REMOVE_PROGRAMS_FINISHED = True


class ConfigurationPanel(QWidget, Ui_ConfigurationPanel):

    msg_error = QtCore.pyqtSignal(str)
    clear_error = QtCore.pyqtSignal()
    ac_changed = QtCore.pyqtSignal(Aircraft)

    def __init__(self, parent=None, *args, **kwargs):
        QWidget.__init__(self, parent=parent, *args, **kwargs)
        self.setupUi(self)
        self.console_widget.filter_widget.hide()
        self.conf = None        # type: conf.Conf
        self.currentAC = None   # type: str
        self.flight_plan_editor = None
        self.header.set_changed.connect(self.handle_set_changed)
        self.header.ac_changed.connect(self.update_ac)
        self.header.id_changed.connect(self.handle_id_changed)
        self.header.refresh_button.clicked.connect(self.refresh_ac)
        self.header.color_button.clicked.connect(self.change_color)
        self.header.save_button.clicked.connect(lambda: self.conf.save())
        self.addAction(self.save_conf_action)
        self.save_conf_action.triggered.connect(lambda: self.conf.save())
        self.conf_widget.conf_changed.connect(self.handle_conf_changed)
        self.conf_widget.setting_changed.connect(self.handle_setting_changed)
        self.conf_widget.flight_plan.edit_alt.connect(self.edit_flightplan_gcs)
        self.header.rename_action.triggered.connect(self.rename_ac)
        self.header.new_ac_action.triggered.connect(self.new_ac)
        self.header.duplicate_action.triggered.connect(self.duplicate_ac)
        self.header.remove_ac_action.triggered.connect(self.remove_ac)
        self.build_widget.spawn_program.connect(self.launch_program)

    def init(self):
        sets = paparazzi.get_list_of_conf_files()
        settings = utils.get_settings()
        self.header.set_sets(sets, conf_init=Conf.get_current_conf())
        last_ac: QtCore.QVariant = settings.value("ui/last_AC", None, str)
        last_target: QtCore.QVariant = settings.value("ui/last_target", None, str)
        if last_ac is not None:
            self.header.ac_combo.setCurrentText(last_ac)
        if last_target is not None:
            self.build_widget.target_combo.setCurrentText(last_target)
        window_size: QtCore.QSize = settings.value("ui/window_size", QtCore.QSize(1000, 600), QtCore.QSize)
        lpw = settings.value("ui/left_pane_width", 100, int)
        self.splitter.setSizes([lpw, window_size.width() - lpw])

    def handle_set_changed(self, conf_file):
        self.conf = Conf(conf_file)
        Conf.set_current_conf(conf_file)
        self.build_widget.set_conf(self.conf)
        acs = [ac.name for ac in self.conf.aircrafts]
        self.header.set_acs(acs)

    def disable_sets(self):
        self.header.set_combo.setDisabled(True)

    def enable_sets(self):
        self.header.set_combo.setDisabled(False)

    def update_ac(self, ac_name):
        ac = self.conf[ac_name]
        if ac_name != "" and ac is not None:
            self.conf_widget.setDisabled(False)
            self.currentAC = ac_name
            status, stderr = ac.update()
            if status != 0:
                self.msg_error.emit(stderr.decode().strip())
            else:
                self.clear_error.emit()
            self.header.set_ac(ac)
            self.conf_widget.set_ac(ac)
            self.build_widget.update_targets(self.conf[ac_name])
            self.ac_changed.emit(self.conf[ac_name])
        else:
            # self.conf_widget.reset()
            self.conf_widget.setDisabled(True)

    def get_current_ac(self) -> str:
        """
        :return: name of the current AC
        """
        return self.currentAC

    def refresh_ac(self):
        self.update_ac(self.currentAC)

    def handle_id_changed(self, id):
        self.conf[self.currentAC].ac_id = id
        if len(self.conf.get_all(id)) > 1:
            self.header.id_spinBox.setStyleSheet("background-color: red;")
        else:
            self.header.id_spinBox.setStyleSheet("background-color: white;")

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

        self.conf[self.currentAC].settings_modules = modules
        self.conf[self.currentAC].settings = settings
        # should we save each time a tiny change is made ? very inefficient !
        # self.conf.save()

    def handle_conf_changed(self):
        self.conf[self.currentAC].airframe = self.conf_widget.airframe.path
        self.conf[self.currentAC].flight_plan = self.conf_widget.flight_plan.path
        self.conf[self.currentAC].radio = self.conf_widget.radio.path
        self.conf[self.currentAC].telemetry = self.conf_widget.telemetry.path
        # reload settings modules, and update UI
        self.update_ac(self.currentAC)

    def add_ac(self, ac: Aircraft):
        self.conf.append(ac)
        self.header.add_ac(ac.name)

    def new_ac(self):
        orig = Aircraft()
        self.create_ac(orig)

    def remove_ac(self):
        button = QMessageBox.question(self, "Remove AC", "Remove AC <strong>{}</strong>?".format(self.currentAC))
        if button == QMessageBox.Yes:
            self.conf.remove(self.conf[self.currentAC])
            self.header.remove_current()

    def duplicate_ac(self):
        orig = self.conf[self.currentAC]
        self.create_ac(orig)

    def create_ac(self, orig):
        ui_dialog = Ui_Dialog()
        dialog = QDialog(parent=self)
        ui_dialog.setupUi(dialog)

        def verify():
            ok = True
            id = ui_dialog.id_spinbox.value()
            name = ui_dialog.name_edit.text()
            if self.conf[id] is not None or id == 0:
                ui_dialog.id_spinbox.setStyleSheet("background-color: red;")
                ok = False
            else:
                ui_dialog.id_spinbox.setStyleSheet("")

            if self.conf[name] is not None or name == "":
                ui_dialog.name_edit.setStyleSheet("background-color: red;")
                ok = False
            else:
                ui_dialog.name_edit.setStyleSheet("")

            return ok

        def accept():
            if verify():
                dialog.accept()

        def reject():
            dialog.reject()

        def duplicate(result):
            if result:
                new_ac = copy.deepcopy(orig)
                name = ui_dialog.name_edit.text()
                ac_id = ui_dialog.id_spinbox.value()
                new_ac.name = name
                new_ac.ac_id = ac_id
                self.add_ac(new_ac)
                self.header.set_current(name)

        ui_dialog.id_spinbox.setValue(self.conf.get_free_id())
        ui_dialog.buttonBox.accepted.connect(accept)
        ui_dialog.buttonBox.rejected.connect(reject)
        ui_dialog.id_spinbox.valueChanged.connect(verify)
        ui_dialog.name_edit.textChanged.connect(verify)
        dialog.finished.connect(duplicate)
        dialog.open()

    def rename_ac(self):
        orig = self.conf[self.currentAC]
        ui_dialog = Ui_Dialog()
        dialog = QDialog(parent=self)
        ui_dialog.setupUi(dialog)
        ui_dialog.name_edit.setText(orig.name)
        ui_dialog.id_spinbox.setValue(orig.ac_id)

        def verify():
            ok = True
            id = ui_dialog.id_spinbox.value()
            name = ui_dialog.name_edit.text()

            acs_name = self.conf.get_all(name)
            if len(acs_name) > 1 or (len(acs_name) == 1 and acs_name[0] != orig):
                ui_dialog.name_edit.setStyleSheet("background-color: red;")
                ok = False
            else:
                ui_dialog.name_edit.setStyleSheet("")

            acs_id = self.conf.get_all(id)
            if len(acs_id) > 1 or (len(acs_id) == 1 and acs_id[0] != orig):
                ui_dialog.id_spinbox.setStyleSheet("background-color: red;")
                ok = False
            else:
                ui_dialog.id_spinbox.setStyleSheet("")

            return ok

        def accept():
            if verify():
                dialog.accept()

        def reject():
            dialog.reject()

        def rename(result):
            if result:
                orig.name = ui_dialog.name_edit.text()
                orig.ac_id = ui_dialog.id_spinbox.value()
                self.header.rename_ac(orig.name)

        ui_dialog.buttonBox.accepted.connect(accept)
        ui_dialog.buttonBox.rejected.connect(reject)
        ui_dialog.id_spinbox.valueChanged.connect(verify)
        ui_dialog.name_edit.textChanged.connect(verify)
        dialog.finished.connect(rename)
        dialog.open()

    def change_color(self):
        ac = self.conf[self.currentAC]
        initial = QtGui.QColor(ac.get_color())
        color = QColorDialog.getColor(initial, self, "AC color")
        if color.isValid():
            color_name = color.name()
            ac.set_color(color_name)
            self.header.set_color(color_name)

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

    def launch_program(self, shortname, cmd, icon):
        pw = ProgramWidget(shortname, cmd, icon, self.programs_widget)
        self.programs_widget.layout().addWidget(pw)
        pw.ready_read_stderr.connect(lambda: self.console_widget.handle_stderr(pw))
        pw.ready_read_stdout.connect(lambda: self.console_widget.handle_stdout(pw))
        pw.finished.connect(lambda c, s: self.console_widget.handle_program_finished(pw, c, s))
        pw.remove.connect(lambda: self.remove_program(pw))
        if REMOVE_PROGRAMS_FINISHED:
            pw.finished.connect(lambda: self.remove_program(pw))
        pw.start_program()

    def remove_program(self, pw: ProgramWidget):
        self.programs_widget.layout().removeWidget(pw)
        self.console_widget.remove_program(pw)
        pw.deleteLater()
