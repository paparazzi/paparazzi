#!/usr/bin/python3
# Copyright (C) 2008-2022 The Paparazzi Team
# released under GNU GPLv2 or later. See COPYING file.
import os
import conf
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui
from configuration_panel import ConfigurationPanel
from operation_panel import OperationPanel
import utils
from typing import Dict
from lxml import etree as ET
from app_settings import AppSettings


class PprzCenter(QMainWindow):
    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent=parent)
        self.setWindowTitle("Paparazzi Center")
        icon = QtGui.QIcon(os.path.join(utils.PAPARAZZI_HOME, "data", "pictures", "penguin_logo.svg"))
        self.setWindowIcon(icon)
        self.addMenu()
        self.tabwidget = QTabWidget(parent=self)
        self.setCentralWidget(self.tabwidget)
        self.configuration_panel = ConfigurationPanel(self.tabwidget)
        self.operation_panel = OperationPanel(self.tabwidget)
        self.tabwidget.addTab(self.configuration_panel, "Configuration")
        self.tabwidget.addTab(self.operation_panel, "Operation")
        self.status_msg = QLabel()
        self.statusBar().addWidget(self.status_msg)
        self.fill_status_bar()
        self.statusBar().show()
        self.configuration_panel.msg_error.connect(self.handle_error)
        self.configuration_panel.clear_error.connect(self.clear_error)
        self.operation_panel.session.program_spawned.connect(self.configuration_panel.disable_sets)
        self.operation_panel.session.programs_all_stopped.connect(self.configuration_panel.enable_sets)
        self.configuration_panel.ac_changed.connect(self.operation_panel.session.set_aircraft)
        self.configuration_panel.splitter.splitterMoved.connect(self.update_left_pane_width)
        settings = utils.get_settings()
        window_size = settings.value("ui/window_size", QtCore.QSize(1000, 600), QtCore.QSize)
        self.resize(window_size)
        self.configuration_panel.init()
        self.operation_panel.session.init()

    def addMenu(self):
        menubar = QMenuBar()
        file_menu = QMenu("&File", menubar)
        help_menu = QMenu("&Help", menubar)
        menubar.addMenu(file_menu)
        menubar.addMenu(help_menu)
        settings_action = QAction("&Edit Settings", file_menu)
        file_menu.addAction(settings_action)
        about_action = QAction("&About", help_menu)
        help_menu.addAction(about_action)

        def edit_settings():
            settings_dialog = AppSettings(self)
            settings_dialog.show()
        settings_action.triggered.connect(edit_settings)
        about_action.triggered.connect(lambda: QMessageBox.about(self, "About Paparazzi", utils.ABOUT_TEXT))

        self.setMenuBar(menubar)

    def closeEvent(self, e: QtGui.QCloseEvent) -> None:
        if self.operation_panel.session.any_program_running():
            self.operation_panel.session.programs_all_stopped.connect(self.close)
            self.operation_panel.session.stop_all()
            e.ignore()
            self.operation_panel.session.programs_all_stopped.connect(self.close)
        else:
            if utils.get_settings().value("always_keep_changes", False, bool):
                self.configuration_panel.conf.save()
            else:
                conf_tree_orig = self.configuration_panel.conf.tree_orig
                conf_tree = self.configuration_panel.conf.to_xml_tree()
                if ET.tostring(conf_tree) != ET.tostring(conf_tree_orig):
                    buttons = QMessageBox.question(self, "Save configuration?",
                                                   "The configuration has changed, do you want to save it?")
                    if buttons == QMessageBox.Yes:
                        self.configuration_panel.conf.save()
                    else:
                        self.configuration_panel.conf.restore_conf()
                        self.configuration_panel.conf.save()
            self.save_gconf()
            e.accept()

    def save_gconf(self):
        settings = utils.get_settings()
        settings.setValue("ui/window_size", self.size())
        settings.setValue("ui/last_AC", self.configuration_panel.get_current_ac())
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
    import sys
    app = QApplication(sys.argv)
    main_window = PprzCenter()
    main_window.show()
    # qApp.aboutToQuit.connect(main_window.quit)
    sys.exit(app.exec_())

