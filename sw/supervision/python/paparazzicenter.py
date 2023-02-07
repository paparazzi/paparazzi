#!/usr/bin/python3
# Copyright (C) 2008-2022 The Paparazzi Team
# released under GNU GPLv2 or later. See COPYING file.
import os
import sys
import signal
import copy
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui
import utils
from lxml import etree as ET
from conf import Conf, Aircraft, ConfError
from app_settings import AppSettings
from generated.ui_supervision_window import Ui_SupervisionWindow
from generated.ui_new_ac_dialog import Ui_NewACDialog


class PprzCenter(QMainWindow, Ui_SupervisionWindow):
    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent=parent)
        self.setupUi(self)
        self.conf: Conf = None
        self.currentAc: Aircraft = None
        icon = QtGui.QIcon(os.path.join(utils.PAPARAZZI_HOME, "data", "pictures", "penguin_logo.svg"))
        self.setWindowIcon(icon)

        self.settings_action.triggered.connect(self.edit_settings)
        self.about_action.triggered.connect(lambda: QMessageBox.about(self, "About Paparazzi", utils.ABOUT_TEXT))

        self.status_msg = QLabel()
        self.statusBar().addWidget(self.status_msg)
        self.fill_status_bar()

        self.header.set_changed.connect(self.handle_set_changed)
        self.header.ac_changed.connect(self.handle_ac_changed)
        self.header.ac_edited.connect(self.handle_ac_edited)
        self.header.ac_rename.connect(self.handle_rename_ac)
        self.header.ac_delete.connect(self.handle_remove_ac)
        self.header.ac_duplicate.connect(self.handle_new_ac)
        self.header.ac_save.connect(lambda _: self.conf.save())
        self.header.ac_new.connect(self.handle_new_ac)

        self.operation_panel.session.program_spawned.connect(self.header.disable_sets)
        self.operation_panel.session.programs_all_stopped.connect(self.header.enable_sets)

        self.configuration_panel.splitter.splitterMoved.connect(self.update_left_pane_width)
        settings = utils.get_settings()
        window_size = settings.value("ui/window_size", QtCore.QSize(1000, 600), QtCore.QSize)
        self.resize(window_size)
        self.configuration_panel.init()
        self.operation_panel.session.init()
        self.header.update_sets()

    def handle_set_changed(self, conf_file):
        self.conf = Conf(conf_file)
        Conf.set_current_conf(conf_file)
        self.configuration_panel.build_widget.set_conf(self.conf)
        acs = [ac.name for ac in self.conf.aircrafts]
        self.header.set_acs(acs)

        # set last AC as current if it exits in the current conf
        settings = utils.get_settings()
        last_ac: QtCore.QVariant = settings.value("ui/last_AC", None, str)
        if last_ac in acs:
            self.handle_ac_changed(last_ac)
        last_target: QtCore.QVariant = settings.value("ui/last_target", None, str)
        if last_target:
            self.configuration_panel.build_widget.target_combo.setCurrentText(last_target)

    def handle_ac_edited(self, ac: Aircraft):
        # check AC ID
        if len(self.conf.get_all(ac.ac_id)) > 1:
            self.header.id_spinBox.setStyleSheet("background-color: red;")
        else:
            self.header.id_spinBox.setStyleSheet("background-color: white;")
        # update ac, then update all widgets
        try:
            ac.update()
            self.clear_error()
        except ConfError as e:
            self.handle_error(e.__str__())
        self.change_ac(ac)

    def handle_ac_changed(self, ac_name):
        ac = self.conf[ac_name]
        if ac is not None:
            self.handle_ac_edited(ac)     # update AC
            self.change_ac(ac)

    def handle_remove_ac(self, ac: Aircraft):
        button = QMessageBox.question(self, "Remove AC", "Remove AC <strong>{}</strong>?".format(ac.name))
        if button == QMessageBox.Yes:
            self.conf.remove(ac)
            self.header.remove_ac(ac)

    def handle_new_ac(self, orig: Aircraft = None):
        if orig is None:
            orig = Aircraft()

        ui_dialog = Ui_NewACDialog()
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
                self.conf.append(new_ac)
                self.header.add_ac(new_ac)
                self.change_ac(new_ac)

        ui_dialog.id_spinbox.setValue(self.conf.get_free_id())
        ui_dialog.buttonBox.accepted.connect(accept)
        ui_dialog.buttonBox.rejected.connect(reject)
        ui_dialog.id_spinbox.valueChanged.connect(verify)
        ui_dialog.name_edit.textChanged.connect(verify)
        dialog.finished.connect(duplicate)
        dialog.open()

    def change_ac(self, ac):
        self.currentAc = ac
        self.header.set_ac(ac)
        self.configuration_panel.set_ac(ac)
        self.operation_panel.session.set_aircraft(ac)
        self.doc_panel.set_aircraft(ac)

    def handle_rename_ac(self, orig: Aircraft):
        ui_dialog = Ui_NewACDialog()
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

    def edit_settings(self):
        settings_dialog = AppSettings(self)
        settings_dialog.show()

    def quit(self, interactive=True):
        quit_accepted = True
        if self.operation_panel.session.any_program_running():
            quit_accepted = False
            self.operation_panel.session.programs_all_stopped.connect(self.close)
            self.operation_panel.session.stop_all()
            self.operation_panel.session.programs_all_stopped.connect(self.close)
        else:
            restore_conf = True
            if utils.get_settings().value("always_keep_changes", False, bool):
                restore_conf = False
            else:
                conf_tree_orig = self.conf.tree_orig
                conf_tree = self.conf.to_xml_tree()
                if ET.tostring(conf_tree) != ET.tostring(conf_tree_orig) and interactive:
                    buttons = QMessageBox.question(self, "Save configuration?",
                                                   "The configuration has changed, do you want to save it?")
                    if buttons == QMessageBox.Yes:
                        restore_conf = False
            if restore_conf:
                self.conf.restore_conf()
            self.conf.save()
            self.save_gconf()
        return quit_accepted

    def closeEvent(self, e: QtGui.QCloseEvent) -> None:
        if self.quit():
            e.accept()
        else:
            e.ignore()

    def save_gconf(self):
        settings = utils.get_settings()
        settings.setValue("ui/window_size", self.size())
        settings.setValue("ui/last_AC", self.currentAc.name)
        settings.setValue("ui/last_session", self.operation_panel.session.get_current_session())

    def update_left_pane_width(self, pos, index):
        utils.get_settings().setValue("ui/left_pane_width", pos)

    def fill_status_bar(self):
        home_widget = QWidget()
        home_lay = QHBoxLayout(home_widget)
        home_lay.addWidget(QLabel("HOME: ", home_widget))
        home_button = QPushButton(utils.PAPARAZZI_HOME, home_widget)
        home_lay.addWidget(home_button)
        home_button.clicked.connect(lambda: utils.open_terminal(utils.PAPARAZZI_HOME))
        self.statusBar().addPermanentWidget(home_widget)
        self.statusBar().addPermanentWidget(utils.make_line(None, True))
        label_version = QLabel("Version={}".format(utils.get_version()))
        self.statusBar().addPermanentWidget(label_version)
        self.statusBar().addPermanentWidget(utils.make_line(None, True))
        label_build = QLabel("Build={}".format(utils.get_build_version()))
        self.statusBar().addPermanentWidget(label_build)

    def handle_error(self, msg):
        self.status_msg.setText(msg)
        self.statusBar().setStyleSheet("background-color: red;")
        # self.statusBar().showMessage(msg)

    def clear_error(self):
        self.status_msg.setText("")
        self.statusBar().setStyleSheet("")


if __name__ == "__main__":

    app = QApplication(sys.argv)

    timer = QtCore.QTimer()
    timer.start(100)
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 100 ms.

    main_window = PprzCenter()
    main_window.show()
    # qApp.aboutToQuit.connect(main_window.quit)

    def sigint_handler(*args):
        """Handler for the SIGINT signal."""
        print("catched SIGINT")
        sys.stderr.write('\r')
        if main_window.quit(False):
            QApplication.quit()

    signal.signal(signal.SIGINT, sigint_handler)

    sys.exit(app.exec_())
