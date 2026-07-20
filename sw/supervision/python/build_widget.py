# Copyright (C) 2008-2022 The Paparazzi Team
# released under GNU GPLv2 or later. See COPYING file.
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
from generated.ui_build import Ui_Build
import lxml.etree as ET
import os
import utils
from conf import Aircraft
from typing import Dict, List, Optional, Set
from dataclasses import dataclass, field
import re


@dataclass
class FlashMode:
    name: str
    vars: Dict[str, str] = field(default_factory=dict)
    boards: List[str] = field(default_factory=list)

    def match(self, board):
        for regex in self.boards:
            if re.match(regex, board) is not None:
                return True
        return False


class BuildWidget(Ui_Build, QWidget):

    multi_action_requested = QtCore.pyqtSignal(str)

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.setupUi(self)
        # uic.loadUi("ui/build.ui", self)
        self.flash_modes: List[FlashMode] = self.parse_flash_modes()
        self.build_button.clicked.connect(self.build)
        self.clean_button.clicked.connect(self.clean)
        self.flash_button.clicked.connect(self.flash)
        self.target_combo.currentTextChanged.connect(lambda _: self.multi_action_requested.emit("RefreshFlashModes"))

    @staticmethod
    def parse_flash_modes() -> List[FlashMode]:
        flash_xml = ET.parse(os.path.join(utils.CONF_DIR, "flash_modes.xml"))
        modes = []
        for xml_mode in flash_xml.getroot().findall("mode"):
            mode_name = xml_mode.get("name")
            vars = {}
            for xml_var in xml_mode.findall("variable"):
                var_name = xml_var.get("name")
                var_value = xml_var.get("value")
                vars[var_name] = var_value
            mode = FlashMode(mode_name, vars)
            for xml_board in xml_mode.find("boards").findall("board"):
                board = xml_board.get("name")
                mode.boards.append(board)
            modes.append(mode)
        return modes

    def get_flash_modes(self, board):
        modes = filter(lambda fm: fm.match(board), self.flash_modes)
        mode_names = [mode.name for mode in modes]
        return mode_names

    def update_targets_for_aircrafts(self, aircrafts: List[Aircraft]):
        selected_target = self.target_combo.currentText()
        self.target_combo.clear()
        self.device_combo.clear()
        if not aircrafts:
            return

        common_targets = set(aircrafts[0].boards.keys())
        for ac in aircrafts[1:]:
            common_targets.intersection_update(ac.boards.keys())
        for target in sorted(common_targets):
            self.target_combo.addItem(target)
        if selected_target in common_targets:
            self.target_combo.setCurrentText(selected_target)
        self.update_flash_modes_for_aircrafts(aircrafts)

    def update_flash_modes_for_aircrafts(self, aircrafts: List[Aircraft]):
        selected_flash_mode = self.device_combo.currentText()
        self.device_combo.clear()
        self.device_combo.addItem("Default")
        target = self.target_combo.currentText()
        if not aircrafts or not target:
            return

        common_modes: Optional[Set[str]] = None
        for ac in aircrafts:
            board = ac.boards.get(target)
            modes = set(self.get_flash_modes(board)) if board is not None else set()
            common_modes = modes if common_modes is None else common_modes.intersection(modes)
        if common_modes:
            self.device_combo.addItems(sorted(common_modes))
        if selected_flash_mode and self.device_combo.findText(selected_flash_mode) >= 0:
            self.device_combo.setCurrentText(selected_flash_mode)

    def get_current_target(self) -> str:
        return self.target_combo.currentText()

    def build(self):
        self.multi_action_requested.emit("Build")

    def clean(self):
        self.multi_action_requested.emit("Clean")

    def flash(self):
        self.multi_action_requested.emit("Flash")

    def enable_buttons(self, enable: bool):
        self.build_button.setEnabled(enable)
        self.clean_button.setEnabled(enable)
        self.flash_button.setEnabled(enable)
