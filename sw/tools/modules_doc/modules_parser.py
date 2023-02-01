from sphinx.application import Sphinx
from sphinx.parsers import Parser
from typing import Dict, Any, List
from sphinx import addnodes, errors
from sphinx.parsers import RSTParser
from docutils import nodes
from lxml import etree as ET
from dataclasses import dataclass


class Define:
    def __init__(self, xml: ET._Element):
        self.name = xml.attrib["name"]
        self.value = xml.get("value", "")
        self.description = xml.get("description", "")


class Section:
    def __init__(self, xml: ET._Element):
        self.name = xml.attrib["name"]
        self.prefix = xml.attrib.get("prefix")
        self.defines = [Define(x) for x in xml.findall("define")]


class ModuleParser:
    def __init__(self, inputstring):
        root = ET.fromstring(inputstring)
        self.name = root.attrib["name"]     # name required
        # self.dir = root.attrib["dir"]     # dir optional!!!
        edoc = root.find("doc")             # doc required
        self.description = edoc.find("description").text    # description required
        self.configures = [Define(xml) for xml in edoc.findall("configure")]
        self.defines = [Define(xml) for xml in edoc.findall("define")]
        self.sections = [Section(xml) for xml in edoc.findall("section")]
        self.depends = []
        self.conflicts = []
        self.provides = []
        edep = root.find("dep")     # type: ET._Element
        if edep is not None:
            edepends = edep.find("depends")     # type: ET._Element
            if edepends is not None:
                self.depends = edepends.text.split(",")
            eprovides = edep.find("provides")
            if eprovides is not None:
                self.provides = eprovides.text.split(",")
            econflicts = edep.find("conflicts")
            if econflicts is not None:
                self.conflicts = econflicts.text.split(",")


class ModuleDoc(Parser):

    supported = ("pprz_module", "xml")

    def make_table_row(self, *args, colwidth=[]):
        row = nodes.row()
        for i, arg in enumerate(args):
            if len(colwidth) > i:
                entry = nodes.entry(morecols=colwidth[i]-1)
            else:
                entry = nodes.entry()
            entry += nodes.paragraph(text=arg)
            row += entry
        return row

    def define_to_table(self, defines, title, section_row=None):
        t = nodes.table()
        t += nodes.title(text=title)
        tgroup = nodes.tgroup(cols=3)
        for _ in range(3):
            colspec = nodes.colspec(colwidth=1)
            tgroup.append(colspec)
        rows = []

        header_row = self.make_table_row("Name", "Value", "Description")
        nt = nodes.thead()
        if section_row is not None:
            nt += section_row
        nt += header_row
        tgroup += nt

        for define in defines:
            row = self.make_table_row(define.name, define.value, define.description)
            rows.append(row)
        tbody = nodes.tbody()
        tbody.extend(rows)
        tgroup += tbody
        t += tgroup
        return t

    def make_section(self, title):
        section = nodes.section()
        section += nodes.title(text=title)
        section["ids"].append(title)
        return section

    def make_list(self, args):
        dl = nodes.bullet_list()
        for arg in args:
            item = nodes.list_item()
            txt = nodes.Text(arg)
            item += txt
            dl += item
        return dl

    def parse(self, inputstring: str, document: addnodes.document):
        self.setup_parse(inputstring, document)
        try:
            m = ModuleParser(inputstring)
        except:
            # print(document.source, document.current_source)
            print(document.reporter.warning("test"))
            # print(document.transformer)
            self.finish_parse()
            return
            pass

        root = self.make_section(m.name)

        # description
        root.append(nodes.paragraph(text=m.description))

        if len(m.configures) > 0:
            root += self.define_to_table(m.configures, "Configures")
        if len(m.defines) > 0:
            header_row = self.make_table_row("Defines", colwidth=[3])
            root += self.define_to_table(m.defines, "Defines")
        if len(m.sections) > 0:
            for s in m.sections:
                header_row = self.make_table_row("{}".format(s.prefix), colwidth=[3])
                root += self.define_to_table(s.defines, "Section {}".format(s.name), header_row)
        if len(m.depends) > 0:
            ds = self.make_section("Depends")
            ds.append(self.make_list(m.depends))
            root.append(ds)
        if len(m.provides) > 0:
            ds = self.make_section("Provides")
            ds.append(self.make_list(m.provides))

            root.append(ds)
        if len(m.conflicts) > 0:
            ds = self.make_section("Conflicts")
            ds.append(self.make_list(m.conflicts))
            root.append(ds)

        if len(m.sections) > 0:
            ds = self.make_section("Sections")

            root.append(ds)

        document.append(root)
        self.finish_parse()


def setup(app: "Sphinx") -> Dict[str, Any]:
    app.add_source_suffix(".xml", "pprz_module")
    app.add_source_parser(ModuleDoc)
