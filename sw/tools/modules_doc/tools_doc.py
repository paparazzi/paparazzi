"""
Sphinx directive ``paparazzi-tools``: one page with every tool of the Paparazzi Center.

The tools are read from conf/tools/*.xml. The description, usage and options come from
the <doc> block of each file, written by ``tests/tools/test_tools.py --update`` from the
--help output of the tool (the documentation server cannot build or start the tools).
"""

import os
import re
import textwrap
import xml.etree.ElementTree as ET
from typing import Any, Dict, List

from docutils import nodes
from docutils.parsers.rst import Directive
from sphinx.application import Sphinx


class ToolInfo:
    def __init__(self, xml_file: str):
        root = ET.parse(xml_file).getroot()
        self.file = os.path.basename(xml_file)
        self.name = root.get("name", self.file)
        self.command = root.get("command", "")
        self.args = [(a.get("flag") or "", a.get("constant")) for a in root.findall("arg")]
        edoc = root.find("doc")
        desc = edoc.findtext("description") if edoc is not None else None
        self.description = textwrap.dedent(desc).strip("\n") if desc else ""
        self.usage = (edoc.findtext("usage") or "").strip() if edoc is not None else ""
        self.options = [(o.get("flag", ""), o.get("description", ""))
                        for o in edoc.findall("option")] if edoc is not None else []

    @property
    def external(self) -> bool:
        return self.command.startswith("$") or not self.command.startswith("sw/")

    @property
    def anchor(self) -> str:
        return "tool-" + re.sub(r"[^a-z0-9]+", "-", self.name.lower()).strip("-")

    def summary(self) -> str:
        """First sentence of the description."""
        first = self.description.split("\n\n")[0].replace("\n", " ").strip()
        m = re.match(r"(.+?[.!?])(\s|$)", first)
        return m.group(1) if m else first


def make_table(header: List[str], rows: List[List[nodes.Node]], widths: List[int]) -> nodes.table:
    table = nodes.table()
    tgroup = nodes.tgroup(cols=len(header))
    for w in widths:
        tgroup += nodes.colspec(colwidth=w)
    thead = nodes.thead()
    row = nodes.row()
    for h in header:
        row += nodes.entry("", nodes.paragraph(text=h))
    thead += row
    tgroup += thead
    tbody = nodes.tbody()
    for cells in rows:
        row = nodes.row()
        for c in cells:
            row += nodes.entry("", c)
        tbody += row
    tgroup += tbody
    table += tgroup
    return table


def description_nodes(text: str) -> List[nodes.Node]:
    """Paragraphs of a help description; indented blocks (diagrams, examples) stay as they are."""
    result = []
    for block in re.split(r"\n\s*\n", text):
        if not block.strip():
            continue
        lines = block.split("\n")
        if any(l[:1].isspace() for l in lines):
            result.append(nodes.literal_block(text=block))
        else:
            result.append(nodes.paragraph(text=" ".join(l.strip() for l in lines)))
    return result


class ToolsDirective(Directive):
    has_content = False

    def run(self) -> List[nodes.Node]:
        env = self.state.document.settings.env
        tools_dir = os.path.normpath(os.path.join(env.app.confdir, "..", "..", "..", "conf", "tools"))
        tools = []
        for f in sorted(os.listdir(tools_dir)):
            if f.endswith(".xml"):
                env.note_dependency(os.path.join(tools_dir, f))
                try:
                    tools.append(ToolInfo(os.path.join(tools_dir, f)))
                except Exception as e:
                    self.state.document.reporter.warning(f"tool {f} not parsed: {type(e).__name__}: {e}")
        tools.sort(key=lambda t: t.name.lower())

        result = []
        groups = [("Paparazzi tools", [t for t in tools if not t.external]),
                  ("External programs", [t for t in tools if t.external])]
        for title, group in groups:
            section = nodes.section(ids=[re.sub(r"[^a-z]+", "-", title.lower())])
            section += nodes.title(text=title)
            rows = []
            for t in group:
                ref = nodes.reference("", t.name, refid=t.anchor)
                rows.append([nodes.paragraph("", "", ref), nodes.paragraph(text=t.summary())])
            section += make_table(["Tool", "Description"], rows, [1, 3])
            for t in group:
                section += self.tool_section(t)
            result.append(section)
        return result

    def tool_section(self, t: ToolInfo) -> nodes.section:
        section = nodes.section(ids=[t.anchor])
        section += nodes.title(text=t.name)
        p = nodes.paragraph()
        p += nodes.strong(text="Command: ")
        p += nodes.literal(text=t.command.lstrip("$"))
        p += nodes.Text(f"  (conf/tools/{t.file})")
        section += p
        if t.args:
            p = nodes.paragraph()
            p += nodes.strong(text="Started by the Center with: ")
            p += nodes.literal(text=" ".join(" ".join(x for x in a if x) for a in t.args))
            section += p
        if t.description:
            section.extend(description_nodes(t.description))
        if t.usage:
            section += nodes.literal_block(text=t.usage)
        if t.options:
            rows = [[nodes.paragraph("", "", nodes.literal(text=f)), nodes.paragraph(text=d)]
                    for f, d in t.options]
            section += make_table(["Option", "Description"], rows, [2, 3])
        if not (t.description or t.usage or t.options):
            section += nodes.paragraph(text="No description yet." if t.external else
                                       "No description yet: the tool did not print a help text.")
        return section


def setup(app: Sphinx) -> Dict[str, Any]:
    app.add_directive("paparazzi-tools", ToolsDirective)
    return {"parallel_read_safe": True, "parallel_write_safe": True}
