import os, sys, copy
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui, QtWidgets
from generated.ui_configuration_panel import Ui_ConfigurationPanel
from generated.ui_new_ac_dialog import Ui_Dialog
from program_widget import ProgramWidget
from conf import *

PAPARAZZI_SRC = os.getenv("PAPARAZZI_SRC")
PAPARAZZI_HOME = os.getenv("PAPARAZZI_HOME", PAPARAZZI_SRC)
lib_path = os.path.normpath(os.path.join(PAPARAZZI_SRC, 'sw', 'lib', 'python'))
sys.path.append(lib_path)
import paparazzi

# TODO make a setting ?
REMOVE_PROGRAMS_FINISHED = True


class ConfigurationPanel(QWidget):

    msg_error = QtCore.pyqtSignal(str)
    clear_error = QtCore.pyqtSignal()

    def __init__(self, parent=None, *args, **kwargs):
        QWidget.__init__(self, parent=parent, *args, **kwargs)
        self.ui = Ui_ConfigurationPanel()
        self.ui.setupUi(self)
        self.ui.console_widget.ui.filter_widget.hide()
        self.conf = None        # type: conf.Conf
        self.currentAC = None   # type: str
        self.ui.header.set_changed.connect(self.handle_set_changed)
        self.ui.header.ac_changed.connect(self.update_ac)
        self.ui.header.id_changed.connect(self.handle_id_changed)
        self.ui.header.ui.refresh_button.clicked.connect(self.refresh_ac)
        self.ui.header.ui.color_button.clicked.connect(self.change_color)
        self.ui.header.ui.save_button.clicked.connect(lambda: self.conf.save())
        self.addAction(self.ui.save_conf_action)
        self.ui.save_conf_action.triggered.connect(lambda: self.conf.save())
        sets = paparazzi.get_list_of_conf_files()
        self.ui.header.set_sets(sets, conf_init=Conf.get_current_conf())
        self.ui.conf_widget.conf_changed.connect(self.handle_conf_changed)
        self.ui.conf_widget.setting_changed.connect(self.handle_setting_changed)
        self.ui.header.ui.rename_action.triggered.connect(self.rename_ac)
        self.ui.header.ui.new_ac_action.triggered.connect(self.new_ac)
        self.ui.header.ui.duplicate_action.triggered.connect(self.duplicate_ac)
        self.ui.header.ui.remove_ac_action.triggered.connect(self.remove_ac)
        self.ui.build_widget.spawn_program.connect(self.launch_program)

    def handle_set_changed(self, conf_file):
        self.conf = Conf(conf_file)
        Conf.set_current_conf(conf_file)
        self.ui.build_widget.set_conf(self.conf)
        acs = [ac.name for ac in self.conf.aircrafts]
        self.ui.header.set_acs(acs)

    def disable_sets(self):
        self.ui.header.ui.set_combo.setDisabled(True)

    def enable_sets(self):
        self.ui.header.ui.set_combo.setDisabled(False)

    def update_ac(self, ac_name):
        ac = self.conf[ac_name]
        if ac_name != "" and ac is not None:
            self.ui.conf_widget.setDisabled(False)
            self.currentAC = ac_name
            status, stderr = ac.update_settings_modules()
            if status != 0:
                self.msg_error.emit(stderr.decode().strip())
            else:
                self.clear_error.emit()
            self.ui.header.set_ac(ac)
            self.ui.conf_widget.set_ac(ac)
            self.ui.build_widget.update_targets(self.conf[ac_name])
        else:
            # self.ui.conf_widget.reset()
            self.ui.conf_widget.setDisabled(True)

    def refresh_ac(self):
        self.update_ac(self.currentAC)

    def handle_id_changed(self, id):
        self.conf[self.currentAC].ac_id = id
        if len(self.conf.get_all(id)) > 1:
            self.ui.header.ui.id_spinBox.setStyleSheet("background-color: red;")
        else:
            self.ui.header.ui.id_spinBox.setStyleSheet("background-color: white;")

    def handle_setting_changed(self):
        def make_setting(item: QListWidgetItem):
            name = item.text()
            state = True if item.checkState() == QtCore.Qt.Checked else False
            return Setting(name, state)

        modules, settings = [], []
        for i in range(self.ui.conf_widget.settings.count()):
            item = self.ui.conf_widget.settings.item(i)
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
        self.conf[self.currentAC].airframe = self.ui.conf_widget.airframe.path
        self.conf[self.currentAC].flight_plan = self.ui.conf_widget.flight_plan.path
        self.conf[self.currentAC].radio = self.ui.conf_widget.radio.path
        self.conf[self.currentAC].telemetry = self.ui.conf_widget.telemetry.path
        # reload settings modules, and update UI
        self.update_ac(self.currentAC)

    def add_ac(self, ac: Aircraft):
        self.conf.append(ac)
        self.ui.header.add_ac(ac.name)

    def new_ac(self):
        orig = Aircraft()
        self.create_ac(orig)

    def remove_ac(self):
        button = QMessageBox.question(self, "Remove AC", "Remove AC <strong>{}</strong>?".format(self.currentAC))
        if button == QMessageBox.Yes:
            self.conf.remove(self.conf[self.currentAC])
            self.ui.header.remove_current()

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
                self.ui.header.set_current(name)

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
                self.ui.header.rename_ac(orig.name)

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
            self.ui.header.set_color(color_name)

    def launch_program(self, shortname, cmd, icon):
        pw = ProgramWidget(shortname, cmd, icon, self.ui.programs_groupbox)
        self.ui.programs_groupbox.layout().addWidget(pw)
        pw.ready_read_stderr.connect(lambda: self.ui.console_widget.handle_stderr(pw))
        pw.ready_read_stdout.connect(lambda: self.ui.console_widget.handle_stdout(pw))
        pw.finished.connect(lambda c, s: self.ui.console_widget.handle_program_finished(pw, c, s))
        pw.remove.connect(lambda: self.remove_program(pw))
        if REMOVE_PROGRAMS_FINISHED:
            pw.finished.connect(lambda: self.remove_program(pw))
        pw.start_program()

    def remove_program(self, pw: ProgramWidget):
        self.ui.programs_groupbox.layout().removeWidget(pw)
        self.ui.console_widget.remove_program(pw)
        pw.deleteLater()
