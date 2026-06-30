#!/usr/bin/env python3
# Copyright (C) 2008-2022 The Paparazzi Team
# released under GNU GPLv2 or later. See COPYING file.
import os
import sys
import utils

lib_path = os.path.normpath(os.path.join(utils.PAPARAZZI_SRC, 'sw', 'lib', 'python'))
sys.path.append(lib_path)


import signal
import copy
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui
from lxml import etree as ET
from conf import Conf, Aircraft, ConfError, common_aircraft_config
from app_settings import AppSettings
from generated.ui_supervision_window import Ui_SupervisionWindow
from generated.ui_new_ac_dialog import Ui_NewACDialog
from program_widget import TabProgramsState
from aircraft_list_widget import AircraftListWidget


dirname = os.path.dirname(os.path.abspath(__file__))
MODE_MONO = "mono"
MODE_MULTI = "multi"


TAB_ICONS = {TabProgramsState.IDLE: QtGui.QIcon(),
             TabProgramsState.RUNNING: QtGui.QIcon(":/icons/icons/running.svg"),
             TabProgramsState.ERROR: QtGui.QIcon(":/icons/icons/error.svg")}


class PprzCenter(QMainWindow, Ui_SupervisionWindow):
    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent=parent)
        self.setupUi(self)
        self.conf: Conf = None
        self.currentAc: Aircraft = None
        self.selection_mode = MODE_MONO
        self.multi_programs_running = 0
        icon = QtGui.QIcon(os.path.join(utils.PAPARAZZI_HOME, "data", "pictures", "penguin_logo.svg"))
        self.setWindowIcon(icon)

        self.settings_action.triggered.connect(self.edit_settings)
        self.about_action.triggered.connect(lambda: QMessageBox.about(self, "About Paparazzi", utils.ABOUT_TEXT))
        self.setup_selection_mode_ui()

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

        self.configuration_panel.build_widget.refresh_ac.connect(self.handle_ac_edited)
        self.configuration_panel.config_file_changed.connect(self.handle_multi_config_file_changed)
        self.configuration_panel.build_widget.multi_action_requested.connect(self.run_multi_build_action)
        self.configuration_panel.build_widget.target_combo.currentTextChanged.connect(
            lambda _: self.refresh_multi_flash_modes()
        )
        self.configuration_panel.program_state_changed.connect(lambda state: self.programs_state_changed(state, 0))
        self.operation_panel.session.program_state_changed.connect(lambda state: self.programs_state_changed(state, 1))

        self.operation_panel.session.program_spawned.connect(self.header.disable_sets)
        self.operation_panel.session.programs_all_stopped.connect(self.header.enable_sets)

        self.operation_panel.session.tools_changed.connect(self.configuration_panel.handle_tools_changed)

        self.configuration_panel.splitter.splitterMoved.connect(self.update_left_pane_width)
        settings = utils.get_settings()
        window_size = settings.value("ui/window_size", QtCore.QSize(1000, 600), QtCore.QSize)
        self.resize(window_size)
        self.set_selection_mode(settings.value("ui/selection_mode", MODE_MONO, str))
        self.configuration_panel.init()
        self.operation_panel.session.init()
        QtCore.QTimer.singleShot(100, self.header.update_sets)

    def setup_selection_mode_ui(self):
        self.header.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.header.setFixedHeight(self.header.sizeHint().height())

        self.selection_splitter = QSplitter(QtCore.Qt.Horizontal, self.centralwidget)
        self.aircraft_list = AircraftListWidget(self.selection_splitter)
        self.aircraft_list.setMinimumWidth(220)
        self.aircraft_list.row_requested.connect(self.handle_aircraft_list_row_requested)
        self.aircraft_list.color_requested.connect(self.change_aircraft_color)
        self.aircraft_list.menu_requested.connect(self.open_aircraft_action_menu)
        self.aircraft_list.template_requested.connect(self.handle_aircraft_list_double_clicked)
        self.aircraft_list.current_aircraft_changed.connect(self.handle_aircraft_list_current_changed)
        self.aircraft_list.selection_changed.connect(self.handle_aircraft_selection_changed)

        self.verticalLayout.removeWidget(self.tabwidget)
        self.selection_splitter.addWidget(self.aircraft_list)
        self.selection_splitter.addWidget(self.tabwidget)
        self.selection_splitter.setSizes([260, 940])
        self.verticalLayout.addWidget(self.selection_splitter)
        self.verticalLayout.setStretch(0, 0)
        self.verticalLayout.setStretch(1, 1)

        self.mode_menu = self.menuBar().addMenu("Mode")
        self.mode_action_group = QActionGroup(self)
        self.mode_action_group.setExclusive(True)
        self.mono_selection_action = QAction("Mono selection", self)
        self.mono_selection_action.setCheckable(True)
        self.multi_selection_action = QAction("Multi selection", self)
        self.multi_selection_action.setCheckable(True)
        self.mode_action_group.addAction(self.mono_selection_action)
        self.mode_action_group.addAction(self.multi_selection_action)
        self.mode_menu.addAction(self.mono_selection_action)
        self.mode_menu.addAction(self.multi_selection_action)
        self.mono_selection_action.triggered.connect(lambda: self.set_selection_mode(MODE_MONO))
        self.multi_selection_action.triggered.connect(lambda: self.set_selection_mode(MODE_MULTI))

    def set_selection_mode(self, mode: str):
        if mode not in (MODE_MONO, MODE_MULTI):
            mode = MODE_MONO
        self.selection_mode = mode
        self.header.setVisible(mode == MODE_MONO)
        self.aircraft_list.setVisible(mode == MODE_MULTI)
        self.configuration_panel.build_widget.set_multi_mode(mode == MODE_MULTI)
        self.mono_selection_action.setChecked(mode == MODE_MONO)
        self.multi_selection_action.setChecked(mode == MODE_MULTI)
        if mode == MODE_MULTI:
            if self.multi_programs_running == 0:
                self.configuration_panel.build_widget.enable_buttons(True)
            self.update_multi_selection_config()
        elif self.currentAc is not None:
            if self.multi_programs_running == 0:
                self.configuration_panel.build_widget.enable_buttons(True)
            self.configuration_panel.set_ac(self.currentAc)
        utils.get_settings().setValue("ui/selection_mode", mode)

    def handle_aircraft_list_current_changed(self, ac: Aircraft):
        if self.selection_mode == MODE_MULTI and ac is not None:
            self.change_ac(ac)
            self.update_multi_selection_config()

    def handle_aircraft_list_row_requested(self, item: QListWidgetItem):
        if self.selection_mode != MODE_MULTI:
            return
        with QtCore.QSignalBlocker(self.aircraft_list.list_widget):
            for row in range(self.aircraft_list.list_widget.count()):
                self.aircraft_list.list_widget.item(row).setCheckState(QtCore.Qt.Unchecked)
            item.setCheckState(QtCore.Qt.Checked)
        self.aircraft_list.update_global_checkbox()
        self.update_multi_selection_config()

    def change_aircraft_color(self, ac: Aircraft):
        initial = QtGui.QColor(ac.get_color())
        color = QColorDialog.getColor(initial, self, "AC color")
        if color.isValid():
            color_name = color.name()
            ac.set_color(color_name)
            if self.currentAc == ac:
                self.header.color_button.setStyleSheet("background-color: {};".format(color_name))
            self.aircraft_list.list_widget.viewport().update()

    def open_aircraft_action_menu(self, ac: Aircraft, pos: QtCore.QPoint):
        menu = QMenu(self)
        rename_action = remove_action = duplicate_action = None
        if ac is not None:
            rename_action = menu.addAction(self.header.rename_action.icon(), self.header.rename_action.text())
            remove_action = menu.addAction(self.header.remove_ac_action.icon(), self.header.remove_ac_action.text())
            duplicate_action = menu.addAction(self.header.duplicate_action.icon(), self.header.duplicate_action.text())
            menu.addSeparator()
        new_action = menu.addAction(self.header.new_ac_action.icon(), self.header.new_ac_action.text())

        action = menu.exec_(pos)
        if action == rename_action and ac is not None:
            self.handle_rename_ac(ac)
        elif action == remove_action and ac is not None:
            self.handle_remove_ac(ac)
        elif action == duplicate_action and ac is not None:
            self.handle_new_ac(ac)
        elif action == new_action:
            self.handle_new_ac()

    def handle_aircraft_list_double_clicked(self, ac: Aircraft):
        if self.selection_mode == MODE_MULTI:
            self.handle_rename_ac(ac)

    def handle_aircraft_selection_changed(self):
        if self.selection_mode == MODE_MULTI:
            self.update_multi_selection_config()

    def update_multi_selection_config(self):
        selected = self.aircraft_list.checked_aircrafts()
        if self.multi_programs_running == 0:
            self.configuration_panel.build_widget.enable_buttons(True)
        if not selected:
            self.configuration_panel.set_ac(None)
            return
        if self.refresh_selected_aircrafts(selected):
            self.refresh_multi_targets()
        else:
            self.configuration_panel.build_widget.update_targets_for_aircrafts([])
        self.configuration_panel.display_config(common_aircraft_config(selected))

    def refresh_selected_aircrafts(self, aircrafts):
        has_error = False
        for ac in aircrafts:
            try:
                ac.update_targets()
                self.aircraft_list.clear_error(ac)
            except ConfError as e:
                has_error = True
                self.aircraft_list.set_error(ac, e.__str__())
                self.handle_error(e.__str__())
        if not has_error:
            self.clear_error()
        return not has_error

    def refresh_multi_targets(self):
        if self.selection_mode != MODE_MULTI:
            return
        self.configuration_panel.build_widget.update_targets_for_aircrafts(self.aircraft_list.checked_aircrafts())

    def refresh_multi_flash_modes(self):
        if self.selection_mode != MODE_MULTI:
            return
        self.configuration_panel.build_widget.update_flash_modes_for_aircrafts(self.aircraft_list.checked_aircrafts())

    def handle_multi_config_file_changed(self, field: str, path: str):
        if self.selection_mode != MODE_MULTI:
            return
        selected = self.aircraft_list.checked_aircrafts()
        for ac in selected:
            setattr(ac, field, path)
            try:
                ac.update()
                self.aircraft_list.clear_error(ac)
                self.clear_error()
            except ConfError as e:
                self.aircraft_list.set_error(ac, e.__str__())
                self.handle_error(e.__str__())
        self.update_multi_selection_config()

    def run_multi_build_action(self, action: str):
        if self.selection_mode != MODE_MULTI:
            return

        selected = self.aircraft_list.checked_aircrafts()
        if not selected:
            QMessageBox.warning(self, "No aircraft", "Select at least one aircraft.")
            return

        make_target = self.get_multi_make_target(action)
        if make_target is None:
            return

        if action in ("Build", "Flash"):
            if not self.refresh_selected_aircrafts(selected):
                return
            has_error = False
            for ac in selected:
                try:
                    ac.update_settings()
                    self.aircraft_list.clear_error(ac)
                except ConfError as e:
                    has_error = True
                    self.aircraft_list.set_error(ac, e.__str__())
                    self.handle_error(e.__str__())
            if has_error:
                return
            self.conf.save(False)

        if action == "Clean":
            self.aircraft_list.clear_build_statuses(selected)
        elif action == "Build":
            for ac in selected:
                self.aircraft_list.set_build_status(ac, "running")
        elif action == "Flash":
            for ac in selected:
                self.aircraft_list.set_flash_status(ac, "running")

        self.configuration_panel.build_widget.enable_buttons(False)
        self.multi_programs_running += len(selected)
        for ac in selected:
            cmd = self.make_multi_command(ac, action, make_target)
            self.configuration_panel.launch_program(
                "{} {}".format(action, ac.name),
                cmd,
                "default_tool_icon.svg",
                lambda exit_code, _exit_status, ac=ac, action=action: self.handle_multi_program_finished(
                    ac, action, exit_code
                ),
                ac
            )

    def get_multi_make_target(self, action: str):
        if action == "Clean":
            return "clean_ac"

        target = self.configuration_panel.build_widget.target_combo.currentText()
        if not target:
            QMessageBox.warning(self, "No target", "No common target is available for the checked aircraft.")
            return None
        utils.get_settings().setValue("ui/last_target", target)
        if action == "Build":
            return "{}.compile".format(target)
        if action == "Flash":
            utils.get_settings().setValue(
                "ui/last_flash_mode", self.configuration_panel.build_widget.device_combo.currentText()
            )
            return "{}.upload".format(target)
        return None

    def make_multi_command(self, ac: Aircraft, action: str, make_target: str):
        cmd = ["make", "-C", utils.PAPARAZZI_HOME, "-f", "Makefile.ac", "AIRCRAFT={}".format(ac.name)]
        if action == "Build" and self.configuration_panel.build_widget.print_config_checkbox.isChecked():
            cmd.append("PRINT_CONFIG=1")
        if action == "Flash":
            flash_mode = self.configuration_panel.build_widget.device_combo.currentText()
            if flash_mode != "Default":
                for mode in self.configuration_panel.build_widget.flash_modes:
                    if mode.name == flash_mode:
                        cmd.extend("{}={}".format(name, value) for name, value in mode.vars.items())
                        break
        cmd.append(make_target)
        return cmd

    def handle_multi_program_finished(self, ac: Aircraft, action: str, exit_code: int):
        if action == "Build":
            self.aircraft_list.set_build_status(ac, "success" if exit_code == 0 else "error")
        elif action == "Flash":
            self.aircraft_list.set_flash_status(ac, "success" if exit_code == 0 else "error")

        self.multi_programs_running = max(0, self.multi_programs_running - 1)
        if self.multi_programs_running == 0:
            self.configuration_panel.build_widget.enable_buttons(True)

    def programs_state_changed(self, state: TabProgramsState, tab_index):
        self.tabwidget.setTabIcon(tab_index, TAB_ICONS[state])

    def handle_set_changed(self, conf_file):
        self.conf = Conf(conf_file)
        Conf.set_current_conf(conf_file)
        self.configuration_panel.build_widget.set_conf(self.conf)
        self.log_widget.set_conf(self.conf)
        acs = [ac.name for ac in self.conf.aircrafts]
        self.header.set_acs(acs)
        self.aircraft_list.set_aircrafts(self.conf.aircrafts)

        # set last AC as current if it exits in the current conf
        settings = utils.get_settings()
        last_ac: QtCore.QVariant = settings.value("ui/last_AC", None, str)
        if last_ac in acs:
            self.handle_ac_changed(last_ac)
        elif self.conf.aircrafts:
            self.change_ac(self.conf.aircrafts[0])
        self.check_only_current_aircraft()
        last_target: QtCore.QVariant = settings.value("ui/last_target", None, str)
        if last_target:
            self.configuration_panel.build_widget.target_combo.setCurrentText(last_target)
        if self.selection_mode == MODE_MULTI:
            self.update_multi_selection_config()

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
            # self.handle_ac_edited(ac)     # update AC ?
            try:
                ac.update_targets()
                self.clear_error()
            except ConfError as e:
                self.handle_error(e.__str__())
            self.change_ac(ac)

    def handle_remove_ac(self, ac: Aircraft):
        button = QMessageBox.question(self, "Remove AC", "Remove AC <strong>{}</strong>?".format(ac.name))
        if button == QMessageBox.Yes:
            self.conf.remove(ac)
            self.header.remove_ac(ac)
            self.refresh_aircraft_list()

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
                self.refresh_aircraft_list()

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
        self.configuration_panel.console_widget.set_aircraft(ac)
        self.operation_panel.session.set_aircraft(ac)
        self.operation_panel.console.set_aircraft(ac)
        self.doc_panel.set_aircraft(ac)

    def check_only_current_aircraft(self):
        if self.currentAc is None or self.currentAc.name not in self.aircraft_list.items:
            return
        with QtCore.QSignalBlocker(self.aircraft_list.list_widget):
            for item in self.aircraft_list.items.values():
                item.setCheckState(QtCore.Qt.Unchecked)
            self.aircraft_list.items[self.currentAc.name].setCheckState(QtCore.Qt.Checked)
        self.aircraft_list.update_global_checkbox()

    def refresh_aircraft_list(self):
        if self.conf is None:
            return
        self.aircraft_list.set_aircrafts(self.conf.aircrafts)
        self.check_only_current_aircraft()
        if self.selection_mode == MODE_MULTI:
            self.update_multi_selection_config()

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
                self.refresh_aircraft_list()

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
        self.magviewer_widget.stop()
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
        settings.setValue("ui/last_control_panel", self.operation_panel.session.get_current_control_panel())

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
    # argument needed for the webviewer to work (documentation)
    # https://stackoverflow.com/questions/75922410/pyqt5-qwebengineview-blank-window
    app = QApplication(sys.argv + ["--disable-seccomp-filter-sandbox"])

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
