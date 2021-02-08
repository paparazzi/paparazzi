#!/usr/bin/env python3
import os
from lxml import etree
from collections import namedtuple


PPRZ_SRC = os.getenv("PAPARAZZI_SRC")
PPRZ_HOME = os.getenv("PAPARAZZI_HOME")
if PPRZ_SRC is None:
    PPRZ_SRC = os.path.realpath(os.path.dirname(os.path.realpath(__file__)) + "/../../..")
    print("PAPARAZZI_SRC not set. Use {} as paparazzi home.".format(PPRZ_SRC))
if PPRZ_HOME is None:
    PPRZ_HOME = PPRZ_SRC

GPLv2 = """ *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>."""

LICENCES = {"GPLv2": GPLv2, None: "", "None": ""}


Periodic = namedtuple('Periodic', ['fun', 'freq', 'start', 'stop', 'autorun'])
Event = namedtuple('Event', ['fun'])
Init = namedtuple('Init', ['fun'])
Datalink = namedtuple('Datalink', ['fun', 'message', 'fun_c'])
Abi = namedtuple('Abi', ['base', 'message', 'params'])


class FilesCreate:
    def __init__(self):
        self.xml = etree.Element("module")
        self.header = ""
        self.src = ""
        self._name = ""
        self._directory = ""
        self._description = ""
        self._author = ""
        self._email = ""
        self.inits = []
        self.periodics = []
        self.events = []
        self.datalinks = []
        self.abi_bindings = []

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, name):
        self._name = name.lower()

    @property
    def directory(self):
        if self._directory == "":
            return self.name
        return self._directory

    @directory.setter
    def directory(self, d):
        self._directory = d.lower()

    @property
    def description(self):
        return self._description

    @description.setter
    def description(self, description):
        self._description = description

    @property
    def author(self):
        return self._author

    @author.setter
    def author(self, author):
        self._author = author

    @property
    def email(self):
        return self._email

    @email.setter
    def email(self, email):
        self._email = "<{}>".format(email)

    def add_init(self, fun):
        fun = fun.strip("()")
        self.inits.append(Init(fun=fun))

    def add_periodic(self, fun, freq, start="", stop="", autorun='LOCK'):
        # add parenthesis to function name
        fun = fun.strip("()")
        start = start.strip("()")
        stop = stop.strip("()")
        self.periodics.append(Periodic(fun=fun, freq=str(freq), start=start, stop=stop, autorun=autorun))

    def add_event(self, fun):
        fun = fun.strip("()")
        self.events.append(Event(fun=fun))

    def add_datalink(self, fun, message):
        splits = fun.split("(", maxsplit=1)
        if len(splits) > 1:
            name, args = splits
            args = filter(lambda x: not x=='', [arg.strip() for arg in args.strip(")").split(",")])
            args = [arg.rsplit(" ", 1) for arg in args]
            fun_c = name + "(" + ", ".join(["{} {}".format(t, n) for t, n in args]) + ")"
            fun_xml = name + "(/*" + ", ".join(["{}".format(n.strip("*")) for t, n in args]) + "*/)"
        else:
            fun_xml = fun + "(buf)"
            fun_c = fun + "(uint8_t* buf)"
        self.datalinks.append(Datalink(fun=fun_xml, message=message, fun_c=fun_c))

    def add_abi(self, base, message, fields):
        fields = [("sender_id", "uint8_t")] + fields
        params = ", ".join(["{} {}".format(t, f) for (f, t) in fields])
        self.abi_bindings.append(Abi(base=base, message=message, params=params))

    def build_xml(self):
        # set name
        self.xml.attrib["name"] = self.name
        # set directory
        self.xml.attrib["dir"] = self.directory
        #set description
        description = etree.Element("description")
        description.text = self.description
        doc = etree.Element("doc")
        doc.append(description)
        self.xml.append(doc)
        # set header
        h_file = etree.Element("file")
        h_file.attrib["name"] = "{}.h".format(self.name)
        header = etree.Element("header")
        header.append(h_file)
        self.xml.append(header)
        # set init
        if len(self.inits) == 0 and len(self.abi_bindings) > 0:
            # add an auto init for Abi bindings
            self.inits.append(Init(fun="{}_abi_init".format(self.name)))
        for init in self.inits:
            init_el = etree.Element("init")
            init_el.attrib["fun"] = init.fun + "()"
            self.xml.append(init_el)
        # set periodic
        for periodic in self.periodics:
            periodic_el = etree.Element("periodic")
            periodic_el.attrib["fun"] = periodic.fun + "()"
            periodic_el.attrib["freq"] = periodic.freq
            if not periodic.start == "":
                periodic_el.attrib["start"] = periodic.start + "()"
            if not periodic.stop == "":
                periodic_el.attrib["stop"] = periodic.stop + "()"
            if not periodic.autorun == "":
                periodic_el.attrib["autorun"] = periodic.autorun
            self.xml.append(periodic_el)
        # set event
        for event in self.events:
            event_el = etree.Element("event")
            event_el.attrib["fun"] = event.fun + "()"
            self.xml.append(event_el)
        # set datalink
        for datalink in self.datalinks:
            datalink_el = etree.Element("datalink")
            datalink_el.attrib["fun"] = datalink.fun
            datalink_el.attrib["message"] = datalink.message
            self.xml.append(datalink_el)
        # set makefile
        c_file = etree.Element("file")
        c_file.attrib["name"] = "{}.c".format(self.name)
        test = etree.Element("test")
        makefile = etree.Element("makefile")
        makefile.append(c_file)
        makefile.append(test)
        self.xml.append(makefile)

    def build_src(self, licence="GPLv2"):
        cop = "/*\n * Copyright (C) {author} {email}\n{licence}\n */".format(
                author=self.author, email=self.email, licence=LICENCES[licence])
        description = "/** @file \"modules/{dir}/{name}.c\"\n * @author {author} {email}\n * {description}\n */".format(
                dir=self.directory, name=self.name, author=self.author, email=self.email, description=self.description)
        include = "#include \"modules/{dir}/{name}.h\"".format(dir=self.directory, name=self.name)
        declarations = ""
        for abi_binding in self.abi_bindings:
            declarations += "#ifndef {}_ID\n#define {}_ID ABI_BROADCAST\n#endif\n\n".format(abi_binding.base.upper(), abi_binding.base.upper())
            declarations += "static abi_event {}_ev;\n\n".format(abi_binding.base)
            declarations += "static void {}_cb({})\n{{\n  // your abi callback code here\n}}\n\n".format(abi_binding.base, abi_binding.params)
        for init in self.inits:
            declarations += "void {}(void)\n{{\n  // your init code here\n".format(init.fun)
            if len(self.abi_bindings) > 0:
                declarations += "\n  // Abi messages bindings\n"
                for abi_binding in self.abi_bindings:
                    declarations += "  AbiBindMsg{0}({1}_ID, &{2}_ev, {2}_cb);\n".format(abi_binding.message, abi_binding.base.upper(), abi_binding.base)
            declarations += "}\n\n"
        for periodic in self.periodics:
            declarations += "void {}(void)\n{{\n  // your periodic code here.\n  // freq = {} Hz\n}}\n\n".format(
                periodic.fun, periodic.freq)
            if not periodic.start == "":
                declarations += "void {}(void)\n{{\n  // your periodic start code here.\n}}\n\n".format(
                    periodic.start)
            if not periodic.stop == "":
                declarations += "void {}(void)\n{{\n  // your periodic stop code here.\n}}\n\n".format(
                    periodic.stop)
        for event in self.events:
            declarations += "void {}(void)\n{{\n  // your event code here\n}}\n\n".format(event.fun)
        for datalink in self.datalinks:
            declarations += "void {}\n{{\n  // {}\n  // your datalink code here\n}}\n\n".format(datalink.fun_c, datalink.message)
        self.src = "{}\n\n{}\n\n{}\n\n{}\n".format(cop, description, include, declarations)

    def build_header(self, licence="GPLv2"):
        cop = "/*\n * Copyright (C) {author} {email}\n{licence}\n */".format(author=self.author, email=self.email, licence=LICENCES[licence])
        description = "/** @file \"modules/{dir}/{name}.h\"\n * @author {author} {email}\n * {description}\n */".format(
                dir=self.directory, name=self.name, author=self.author, email=self.email, description=self.description)
        includes = ""
        if len(self.datalinks) > 0:
            includes += "#include \"stdint.h\"\n\n"
        declarations = ""
        for init in self.inits:
            declarations += "extern void {}(void);\n".format(init.fun)
        for periodic in self.periodics:
            declarations += "extern void {}(void);\n".format(periodic.fun)
            if not periodic.start == "":
                declarations += "extern void {}(void);\n".format(periodic.start)
            if not periodic.stop == "":
                declarations += "extern void {}(void);\n".format(periodic.stop)
        for event in self.events:
            declarations += "extern void {}(void);\n".format(event.fun)
        for datalink in self.datalinks:
            declarations += "extern void {};\t// {}\n".format(datalink.fun_c, datalink.message)
        self.header = "{cop}\n\n{des}\n\n#ifndef {guard}_H\n#define {guard}_H\n\n{includes}{decl}\n#endif  // {guard}_H\n".format(
            cop=cop, des=description, guard=self.name.upper(), includes=includes, decl=declarations)

    def build_files(self, licence="GPLv2"):
        self.build_xml()
        self.build_src(licence)
        self.build_header(licence)

    def get_filenames(self):
        dir_path = "{}/sw/airborne/modules/{}".format(PPRZ_SRC, self.directory)
        xml_filename = "{}/conf/modules/{}.xml".format(PPRZ_HOME, self.name)
        h_filename = "{}/{}.h".format(dir_path, self.name)
        c_filename = "{}/{}.c".format(dir_path, self.name)
        return xml_filename, h_filename, c_filename, dir_path

    def write_files(self):
        xml_filename, h_filename, c_filename, dir_path = self.get_filenames()

        # XML file
        with open(xml_filename, 'wb') as xml_file:
            xml = etree.tostring(self.xml, pretty_print=True, doctype="<!DOCTYPE module SYSTEM \"module.dtd\">")
            xml_file.write(xml)

        # create directory is needed
        if not os.path.isdir(dir_path):
            os.mkdir(dir_path)

        # create C file
        with open(c_filename, 'w') as c_file:
            c_file.write(self.src)

        # create H file
        with open(h_filename, 'w') as h_file:
            h_file.write(self.header)
        return [xml_filename, h_filename, c_filename]
