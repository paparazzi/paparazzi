from PyQt5.QtWidgets import QWidget
from PyQt5 import QtCore
from generated.ui_build import Ui_Build
import lxml.etree as ET
import os
import utils
from conf import Aircraft, Conf
from typing import List, Dict
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


class BuildWidget(QWidget):

    spawn_program = QtCore.pyqtSignal(str, list, str)

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.ui = Ui_Build()
        self.ui.setupUi(self)
        self.ac: Aircraft = None
        self.conf: Conf = None      # to save conf before building
        self.flash_modes: List[FlashMode] = self.parse_flash_modes()
        self.boards: Dict[str, str] = {}  # {target: board}
        self.ui.build_button.clicked.connect(self.build)
        self.ui.clean_button.clicked.connect(self.clean)
        self.ui.flash_button.clicked.connect(self.flash)
        self.ui.target_combo.currentTextChanged.connect(self.update_flash_mode)

    def set_conf(self, conf: Conf):
        self.conf = conf

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

    def update_targets(self, ac: Aircraft):
        self.ac = ac
        self.ui.target_combo.clear()
        self.boards.clear()

        airframe_xml = ET.parse(os.path.join(utils.CONF_DIR, ac.airframe))
        # for firmware_xml in airframe_xml.getroot().findall("firmware"):
        #     firmware = firmware_xml.get("name")
        #     for target_xml in firmware_xml.findall("target"):
        #         target = target_xml.get("name")
        #         txt = "{} {}".format(firmware, target)
        #         self.ui.target_combo.addItem(txt)
        for firmware_xml in airframe_xml.getroot().findall("firmware"):
            for target_xml in firmware_xml.findall("target"):
                target = target_xml.get("name")
                board = target_xml.get("board")
                self.boards[target] = board
                self.ui.target_combo.addItem(target)

    def update_flash_mode(self, target):
        self.ui.device_combo.clear()
        if target != "":
            self.ui.device_combo.addItem("Default")
            board = self.boards[target]
            flash_modes = self.get_flash_modes(board)
            self.ui.device_combo.addItems(flash_modes)

    def build(self):
        target = self.ui.target_combo.currentText()
        cmd = ["make", "-C", utils.PAPARAZZI_HOME, "-f", "Makefile.ac",
               "AIRCRAFT={}".format(self.ac.name), "{}.compile".format(target)]
        shortname = "Build {}".format(self.ac.name)
        self.conf.save(False)
        self.spawn_program.emit(shortname, cmd, None)

    def clean(self):
        cmd = ["make", "-C", utils.PAPARAZZI_HOME, "-f", "Makefile.ac",
               "AIRCRAFT={}".format(self.ac.name), "clean_ac"]
        shortname = "Clean {}".format(self.ac.name)
        self.spawn_program.emit(shortname, cmd, None)

    def flash(self):
        target = self.ui.target_combo.currentText()
        vars = []
        flash_mode = self.ui.device_combo.currentText()
        if flash_mode != "Default":
            for mode in self.flash_modes:
                if mode.name == flash_mode:
                    vars = ["{}={}".format(var_name, var_value) for (var_name, var_value) in mode.vars.items()]
                    break
            else:
                raise Exception("Flash mode {} not found!".format(flash_mode))
        cmd = ["make", "-C", utils.PAPARAZZI_HOME, "-f", "Makefile.ac",
               "AIRCRAFT={}".format(self.ac.name)] + vars + ["{}.upload".format(target)]
        shortname = "Flash {}".format(self.ac.name)
        self.spawn_program.emit(shortname, cmd, None)

