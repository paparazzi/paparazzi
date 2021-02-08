#!/usr/bin/env python3
#
# Copyright (C) 2021 Fabien Bonneval <fabien.bonneval@enac.fr>
#                    Gautier Hattenberger <gautier.hattenberger@enac.fr>
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.
#

from lxml import etree
from typing import List, Optional, Tuple, Dict
import re
from os import path, getenv
import os
import argparse
import subprocess
import shlex

import TAP

PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../')))

ARCHS = ["chibios", "linux", "sim", "stm32"]

ALT_DIRS = ["../../var/share", "../ext"]

GCC_PARAMS: List[str] = ["-c", "-o", "/dev/null", "-W", "-Wall"]
OTHER_PARAMS: List[str] = [
    "-I./", "-I../include",
    "-I../../var/include",
    "-I../../tests/modules",
    "-Imodules", "-DPERIODIC_TELEMETRY",
    "-DDOWNLINK",
    "-DBOARD_CONFIG=\"../../tests/modules/dummy.h\""]


class Test:
    def __init__(self, tst, files, files_arch, test_name):
        self.configure_regex = re.compile(r"(\$\([a-zA-Z_][a-zA-Z_0-9]*\))")
        self.files = files              # type: List[str]
        self.files_arch = files_arch    # type: List[str]
        self.firmware = None            # type: Optional[str]
        self.archs = []                 # type: List[str]
        self.defines = []               # type: List[Tuple[str,str, str]]
        self.configures = {}            # type: Dict[str:str]
        self.includes = []              # type: List[str]
        self.shells = []                # type: List[str]
        self.test_name = test_name

        self.parse(tst)

    def parse(self, tst):
        self.firmware = tst.attrib.get("firmware")
        archs = tst.attrib.get("arch")
        if archs is None:
            # self.archs = ARCHS
            self.archs = []
        else:
            self.archs = archs.split("|")
            if len(self.archs) == 1:
                if self.archs[0] == "":
                    self.archs = []
                elif self.archs[0][0] == "!":
                    excluded = self.archs[0][1:]
                    self.archs = ARCHS
                    self.archs.remove(excluded)
                    print(f"archs : {self.archs}")

        for define in tst.findall("define"):
            def_name = define.attrib["name"]
            def_val = define.attrib.get("value")
            def_type = define.attrib.get("type")
            self.defines.append((def_name, def_val, def_type))

        for configure in tst.findall("configure"):
            conf_name = configure.attrib["name"]
            conf_value = configure.attrib["value"]
            self.configures[conf_name] = conf_value

        for include in tst.findall("include"):
            self.includes.append(include.attrib["name"])

        for shell in tst.findall("shell"):
            self.shells.append(shell.attrib["cmd"])

        if self.firmware is not None:
            self.configures["SRC_FIRMWARE"] = f"firmwares/{self.firmware}"
            self.includes.append(f"firmwares/{self.firmware}")

    def substitute_configures(self):
        def substitute(string: str):
            if string is None:
                return None
            for m in re.findall(self.configure_regex, string):
                if m[2:-1] in self.configures.keys():
                    string = string.replace(m, self.configures[m[2:-1]])
            return string
        self.files = map(substitute, self.files)
        self.files_arch = map(substitute, self.files_arch)
        self.defines = map(lambda x: (substitute(x[0]), substitute(x[1]), x[2]), self.defines)
        self.includes = map(substitute, self.includes)

    def make_commands(self) -> List[List[str]]:
        self.substitute_configures()
        self.files = list(self.files)
        self.files_arch = list(self.files_arch)

        gcc = "gcc"
        for f in self.files + self.files_arch:
            if re.match("cpp$", f) is not None:
                gcc = "g++"

        defines = list(map(self.define_str, self.defines))
        includes = list((map(self.include_str, self.includes)))
        shells = []
        for shell in self.shells:
            shell_args = shlex.split(shell)
            p = subprocess.run(shell_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
            if p.returncode == 0:
                shells += shlex.split(p.stdout)
            else:
                print(p.stderr)
                return [["exit", "1"]]

        cmds = []
        for file in self.files:
            # build with the "test" arch.
            arch_include = f"-I../../tests/modules/test_arch"
            cmd_args = [gcc] + GCC_PARAMS + [file] + OTHER_PARAMS + defines + includes + [arch_include] + shells
            cmds.append(cmd_args)

        for file in self.files_arch:
            for arch in self.archs:
                file_path = f"arch/{arch}/{file}"
                arch_include = f"-Iarch/{arch}"
                cmd_args = [gcc] + GCC_PARAMS + [file_path] + OTHER_PARAMS + defines + includes + [arch_include] + shells
                # TODO: uncomment to build <file_arch/> files.
                # cmds.append(cmd_args)
        return cmds

    def run(self):
        cmds = self.make_commands()
        diagnostic = []
        returncode = 0
        for cmd_args in cmds:
            p = subprocess.run(cmd_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
            if p.stderr != "":
                # diagnostic.append(str(cmd_args))
                diagnostic.append(p.stderr)

            if p.returncode != 0:
                returncode = 1
        return returncode, self.test_name, diagnostic


    @staticmethod
    def define_str(define: Tuple[str, Optional[str], Optional[str]]) -> str:
        name, value, _type = define
        if value is not None:
            if _type is not None and _type == "string":
                return f'-D{name}="{value}"'
            else:
                return f"-D{name}={value}"
        else:
            return f"-D{name}"

    @staticmethod
    def include_str(name: str) -> str:
        return f"-I{name}"

    def __repr__(self):
        res = ""
        res += "Test{\n"
        res += "\tdefines: "
        for define in self.defines:
            res += define[0] + "=" + (define[1] if define[1] is not None else "None") + " "
        res += "\n\tconfigures: "
        for key, value in self.configures.items():
            res += key + "=" + (value if value is not None else "None") + " "
        res += "\n\tincludes: " + " ".join(self.includes)
        res += "\n\tfiles: "
        res += " ".join(self.files)
        res += "}"
        return res


class Module:

    def __init__(self, filename):
        m_tree = etree.parse(filename)
        mod_elt = m_tree.getroot()
        self.name = mod_elt.attrib["name"]
        # if dir is not specified, the name of the module is used as default directory name
        self.dir = mod_elt.attrib.get("dir", self.name)
        self.tests = []

        for mkf in mod_elt.findall("makefile"):
            files = self.get_files(mkf)
            files_arch = self.get_files_arch(mkf)
            for i, tst in enumerate(mkf.findall("test")):
                test = Test(tst, files, files_arch, f"{self.name}_{i}")
                self.tests.append(test)

    def get_files(self, mkf):
        files = []
        for file in mkf.findall("file"):
            name = file.attrib["name"]
            dir = file.attrib.get("dir")
            if dir is None:
                file_path = "/".join(["modules", self.dir, name])
            else:
                file_path = "/".join([dir, name])
            if not path.exists(file_path):
                for alt_dir in ALT_DIRS:
                    alt_path = f"{alt_dir}/{file_path}"
                    if path.exists(alt_path):
                        file_path = alt_path
                        break

            files.append(file_path)
        return files

    def get_files_arch(self, mkf):
        files = []
        for file in mkf.findall("file_arch"):
            name = file.attrib["name"]
            dir = file.attrib.get("dir")
            if dir is None:
                file_path = "/".join(["modules", self.dir, name])
            else:
                file_path = "/".join([dir, name])
            files.append(file_path)
        return files


def get_modules():
    files = os.listdir(PPRZ_HOME+"/conf/modules")

    def is_xml(filename):
        filename, extension = os.path.splitext(filename)
        return extension == ".xml"

    files = filter(is_xml, files)
    return files


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="test modules")
    parser.add_argument('--file', '-f', default=None, help="module to be tested")
    parser.add_argument('-w', action='store_false')
    args = parser.parse_args()

    def is_xml(filename):
        filename, extension = os.path.splitext(filename)
        return extension == ".xml"

    if args.file is None:
        files = os.listdir(PPRZ_HOME+"/conf/modules")
        files = map(lambda name: PPRZ_HOME+"/conf/modules/"+name, files)
    else:
        files = [args.file]

    files = filter(is_xml, files)

    all_tests = []
    os.chdir(f"{PPRZ_HOME}/sw/airborne")
    for f in files:
        mod = Module(f)
        all_tests += mod.tests

    ok = TAP.Builder.create(len(all_tests)).ok
    for test in all_tests:
        returncode, comment, diagnotics = test.run()
        ok(not returncode, comment)
        if args.w or returncode:
            for d in diagnotics:
                lines = d.split("\n")
                for line in lines:
                    print(f"# {line}")
