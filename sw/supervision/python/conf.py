# Copyright (C) 2008-2022 The Paparazzi Team
# released under GNU GPLv2 or later. See COPYING file.
from dataclasses import dataclass, field
from typing import List, Dict
import lxml.etree as ET
import os
import subprocess
import utils

MOD_DEP = os.path.join(utils.PAPARAZZI_SRC, "sw", "tools", "generators", "dump_modules_list.out")
CONF = os.path.join(utils.PAPARAZZI_HOME, "conf", "conf.xml")


class ConfError(Exception):
    ...


@dataclass
class Setting:
    name: str
    enabled: bool

    def __str__(self):
        if self.enabled:
            return self.name
        else:
            return f"[{self.name}]"


@dataclass
class Aircraft:
    name: str = ""
    ac_id: int = 0
    airframe: str = ""
    radio: str = ""
    telemetry: str = ""
    flight_plan: str = ""
    gui_color: str = "red"
    settings: List[Setting] = field(default_factory=list)
    settings_modules: List[Setting] = field(default_factory=list)
    boards: Dict[str, str] = field(default_factory=dict, init=False)    # {target: board}

    def get_color(self) -> str:
        if self.gui_color.startswith("#"):
            r = self.gui_color[1:3]
            g = self.gui_color[5:7]
            b = self.gui_color[9:11]
            color = "#{}{}{}".format(r, g, b)
            return color

        else:
            return self.gui_color

    def set_color(self, color: str):
        if color.startswith("#"):
            r = color[1:3]
            g = color[3:5]
            b = color[5:7]
            self.gui_color = "#{}00{}00{}00".format(r, g, b)
        else:
            self.gui_color = color

    def update(self):
        self.update_targets()
        self.update_settings()

    def update_settings(self):
        completed = subprocess.run([MOD_DEP, "-ac", self.name, "-af", self.airframe, "-fp", self.flight_plan],
                                   capture_output=True)
        if completed.returncode != 0:
            raise ConfError(completed.stderr.decode().strip())

        def make_setting(m):
            setting = Setting(m, True)
            for s in self.settings_modules:
                if m == s.name and not s.enabled:
                    setting.enabled = False
            return setting

        new_settings_modules = []
        for module_path in completed.stdout.decode().strip().split():
            module = utils.remove_prefix(module_path, utils.CONF_DIR)
            xml = ET.parse(module_path)
            for xml_setting in xml.getroot().findall("settings"):
                name = xml_setting.get("name")
                if name is None:
                    txt = module
                else:
                    txt = "{}~{}~".format(module, name)
                setting = make_setting(txt)
                new_settings_modules.append(setting)

        self.settings_modules = new_settings_modules

    def to_xml(self) -> ET.Element:
        xml = ET.Element("aircraft")
        xml.set("name", self.name)
        xml.set("ac_id", str(self.ac_id))
        xml.set("airframe", self.airframe)
        xml.set("radio", self.radio)
        xml.set("telemetry", self.telemetry)
        xml.set("flight_plan", self.flight_plan)
        settings_modules = " ".join(str(setting) for setting in self.settings_modules)
        settings = " ".join(str(setting) for setting in self.settings)
        xml.set("settings", settings)
        xml.set("settings_modules", settings_modules)
        xml.set("gui_color", self.gui_color)
        return xml

    def to_string(self):
        xml = "  <aircraft\n"
        xml += "   name=\"{}\"\n".format(self.name)
        xml += "   ac_id=\"{}\"\n".format(self.ac_id)
        xml += "   airframe=\"{}\"\n".format(self.airframe)
        xml += "   radio=\"{}\"\n".format(self.radio)
        xml += "   telemetry=\"{}\"\n".format(self.telemetry)
        xml += "   flight_plan=\"{}\"\n".format(self.flight_plan)
        xml += "   settings=\"{}\"\n".format(" ".join(str(setting) for setting in self.settings))
        xml += "   settings_modules=\"{}\"\n".format(" ".join(str(setting) for setting in self.settings_modules))
        xml += "   gui_color=\"{}\"\n".format(self.gui_color)
        xml += "  />"
        return xml

    def update_targets(self):
        self.boards = {}
        try:
            airframe_xml = ET.parse(os.path.join(utils.CONF_DIR, self.airframe))
            for firmware_xml in airframe_xml.getroot().findall("firmware"):
                for target_xml in firmware_xml.findall("target"):
                    target = target_xml.get("name")
                    board = target_xml.get("board")
                    self.boards[target] = board
        except OSError as e:
            raise ConfError("OSError, file {} probably not found!".format(self.airframe))
        except ET.XMLSyntaxError as e:
            raise ConfError("XMLSyntaxError, file {} is illformed !".format(self.airframe))


class Conf:
    def __init__(self, file: str):
        self.file = file        # type: str
        self.aircrafts = []     # type: List[Aircraft]
        xml = ET.parse(os.path.join(utils.CONF_DIR, file))
        self.parse(xml)
        self.tree_orig = self.to_xml_tree()

    def parse(self, conf_tree):
        self.aircrafts.clear()
        for ac_xml in conf_tree.getroot().findall("aircraft"):
            name = ac_xml.attrib["name"]
            ac_id = int(ac_xml.attrib["ac_id"])
            airframe = ac_xml.attrib["airframe"]
            radio = ac_xml.attrib["radio"]
            telemetry = ac_xml.attrib["telemetry"]
            flight_plan = ac_xml.attrib["flight_plan"]
            gui_color = ac_xml.attrib["gui_color"]

            def make_setting(txt):
                name = txt.strip('[]')
                enabled = True if len(name) == len(txt) else False
                return Setting(name, enabled)

            settings = list(map(make_setting, ac_xml.attrib["settings"].split()))
            settings_modules = list(map(make_setting, ac_xml.attrib["settings_modules"].split()))
            ac = Aircraft(name, ac_id, airframe, radio, telemetry, flight_plan,
                          gui_color, settings, settings_modules)
            self.aircrafts.append(ac)

    def __getitem__(self, item):
        for ac in self.aircrafts:
            if (isinstance(item, int) and item == ac.ac_id) or (isinstance(item, str) and item == ac.name):
                return ac

    def __len__(self):
        return len(self.aircrafts)

    def remove(self, ac: Aircraft):
        self.aircrafts.remove(ac)

    def append(self, ac: Aircraft):
        self.aircrafts.append(ac)

    def get_all(self, id):
        acs = []
        for ac in self.aircrafts:
            if id == ac.ac_id or id == ac.name:
                acs.append(ac)
        return acs

    def get_free_id(self):
        ids = [ac.ac_id for ac in self.aircrafts]
        for i in range(1, 256):
            if i not in ids:
                return i

    def to_xml_tree(self):
        conf_xml = ET.Element("conf")
        for ac in self.aircrafts:
            ac_xml = ac.to_xml()
            conf_xml.append(ac_xml)
        tree = ET.ElementTree(conf_xml)
        return tree

    def save(self, refresh_orig=True):
        conf_path = os.path.join(utils.CONF_DIR, self.file)
        with open(conf_path, "w") as fic:
            fic.write(self.to_string())
        print("conf saved to {}".format(conf_path))
        if refresh_orig:
            self.tree_orig = self.to_xml_tree()

    def restore_conf(self):
        print("conf restored.")
        self.parse(self.tree_orig)

    def to_string(self):
        xml = "<conf>\n"
        xml += "\n".join([ac.to_string() for ac in self.aircrafts])
        xml += "\n</conf>\n"
        return xml

    @staticmethod
    def get_current_conf():
        if os.path.islink(CONF):
            #return os.path.realpath(CONF)
            return os.readlink(CONF)
        else:
            return CONF

    @staticmethod
    def set_current_conf(conf: str):
        if not os.path.islink(CONF):
            print("your conf.xml is not a link, it will be overwritten.")
        try:
            os.symlink(conf, CONF)
        except OSError as e:
            os.remove(CONF)
            os.symlink(conf, CONF)





