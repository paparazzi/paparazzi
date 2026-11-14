#!/usr/bin/env python3
#
# Copyright (C) 2026 The Paparazzi Team
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

"""
Test the tools of the Paparazzi Center (conf/tools/*.xml).

Each tool that is a Paparazzi program (its command is a file under sw/) is started
with --help. The help text is split into a usage line, a description and a list of
options, printed as TAP comments and saved in var/tools_help.json.

The <doc> block of each tool xml holds the same information for the documentation
(doc/sphinx, page "Paparazzi Center tools"), because the documentation server cannot
build and start the tools. With --update, this script writes the usage and options of
every tool that printed its help into the <doc> block. A <description> written by hand
is kept; when there is none, the description of the help text is used.

Test results:
 - ok: the tool printed its help and its <doc> block matches it
 - not ok: the tool did not stop with a help text, or its <doc> block is out of date
 - skip: external program (command starting with '$'), or a Python module is missing
"""

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import textwrap
import xml.etree.ElementTree as ET
from os import path
from xml.sax.saxutils import escape, quoteattr

sys.path.insert(0, path.join(path.dirname(path.abspath(__file__)), "..", "modules"))
import TAP  # noqa: E402

PPRZ_HOME = os.getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), "../../")))
PPRZ_SRC = os.getenv("PAPARAZZI_SRC", PPRZ_HOME)
TOOLS_DIR = path.join(PPRZ_HOME, "conf", "tools")
# private Ivy bus, so a tool that ignores --help does not talk to running programs
IVY_BUS = "127.255.255.255:2099"

OPTION_RE = re.compile(r"^\s{0,12}(-{1,2}[A-Za-z0-9?])")
HEADING_RE = re.compile(r"^\S.*:\s*$")
SECTION_HEADINGS = ("options", "optional arguments", "positional arguments", "arguments")
HELP_FLAGS = ("-h", "-help", "--help", "-h, --help", "-h --help", "-help, --help")
NOISE_RE = re.compile(r"invalid option|Unknown option")


def parse_help(text):
    """Split a help text into usage, description, positional arguments and options."""
    usage = []
    header = []       # description lines, "" between paragraphs
    options = []      # [flags, description, whole line taken as flags]
    positionals = []  # [name, description]
    section = None
    current = None    # entry that continuation lines are added to
    in_usage = False
    for line in text.splitlines():
        line = line.rstrip()
        stripped = line.strip()
        if not stripped:
            current = None
            in_usage = False
            if header and header[-1] != "":
                header.append("")
            continue
        if stripped.lower().startswith("usage"):
            usage.append(stripped)
            in_usage = True
            continue
        if in_usage and line[:1].isspace() and not OPTION_RE.match(line):
            usage.append(stripped)
            continue
        in_usage = False
        if HEADING_RE.match(line):
            name = stripped[:-1].strip().lower()
            if name in SECTION_HEADINGS:
                section = name
                current = None
                continue
        if OPTION_RE.match(line):
            parts = re.split(r"\s{2,}|\t+", stripped, maxsplit=1)
            # without a wide gap the whole line is taken as flags; split later if no
            # description follows on the next lines (argparse puts it there)
            current = parts + [""] if len(parts) == 1 else parts
            current.append(len(parts) == 1)
            options.append(current)
            continue
        if line[:1].isspace() and current is not None:
            current[1] = (current[1] + " " + stripped).strip()
            current[2] = False
            continue
        if section == "positional arguments" and line[:1].isspace():
            parts = re.split(r"\s{2,}", stripped, maxsplit=1)
            current = [parts[0], parts[1] if len(parts) > 1 else ""]
            positionals.append(current)
            continue
        current = None
        if not NOISE_RE.search(line):
            header.append(line)
    for opt in options:
        if opt[2]:
            # OCaml Arg style: "-flag <arg> description" with single spaces
            words = opt[0].split(" ")
            flags, rest = words[0], words[1:]
            if rest and rest[0].startswith("<"):
                while rest and not flags.endswith(">"):
                    flags += " " + rest.pop(0)
            opt[0], opt[1] = flags, " ".join(rest)
    options = [opt[:2] for opt in options if opt[0].rstrip(",") not in HELP_FLAGS]
    description = "\n".join(header).strip("\n")
    usage = re.sub(r"\s*Options\s*:$", "", " ".join(usage))
    if usage.lower() == "usage:":  # OCaml Arg prints only "Usage:"
        usage = ""
    return {"usage": usage, "description": description,
            "positionals": [p[:2] for p in positionals], "options": options}


def failure_reason(output, returncode, timed_out):
    """Short reason why a tool did not print its help."""
    patterns = [
        r"(ModuleNotFoundError: No module named '[^']+')",
        r"(ImportError: .*)",
        r"(error while loading shared libraries: .*)",
        r"([Cc]annot open display.*)",
        r"(No such file or directory.*)",
        r"(Permission denied.*)",
    ]
    for p in patterns:
        m = re.search(p, output)
        if m:
            return m.group(1).strip()
    if "Traceback" in output:
        last = [l for l in output.splitlines() if l.strip()][-1]
        return "Python exception: " + last.strip()
    if timed_out:
        return "still running after the timeout (ignores --help?)"
    if returncode < 0:
        return "killed by signal %s" % signal.Signals(-returncode).name
    if not output.strip():
        return "no output, exit code %d" % returncode
    return "exit code %d" % returncode


def read_doc(root):
    """The <doc> block of a tool xml as a dict (None without <doc>)."""
    edoc = root.find("doc")
    if edoc is None:
        return None
    desc = edoc.findtext("description")
    return {"description": textwrap.dedent(desc).strip("\n").rstrip() if desc else None,
            "usage": (edoc.findtext("usage") or "").strip(),
            "options": [[o.get("flag", ""), o.get("description", "")] for o in edoc.findall("option")]}


def doc_block(description, usage, options):
    """Text of a <doc> block, indented for a tool xml."""
    lines = ["  <doc>"]
    if description:
        lines.append("    <description>")
        lines += [("      " + escape(l)).rstrip() for l in description.split("\n")]
        lines.append("    </description>")
    if usage:
        lines.append("    <usage>%s</usage>" % escape(usage))
    for flags, desc in options:
        lines.append("    <option flag=%s description=%s/>" % (quoteattr(flags), quoteattr(desc)))
    lines.append("  </doc>")
    return "\n".join(lines)


def write_doc(xml_file, block):
    """Put a <doc> block right after the <program> tag, replacing the old one."""
    with open(xml_file) as f:
        text = f.read()
    text = re.sub(r"\n?[ \t]*<doc>.*?</doc>[ \t]*", "", text, flags=re.S)
    m = re.search(r"<program\b[^>]*?(/?)>", text, flags=re.S)
    if m.group(1):  # <program .../> without children
        opening = m.group(0)[:-2].rstrip() + ">"
        text = text[:m.start()] + opening + "\n" + block + "\n</program>" + text[m.end():]
    else:
        text = text[:m.end()] + "\n" + block + text[m.end():]
    with open(xml_file, "w") as f:
        f.write(text)


class Tool:
    def __init__(self, xml_file):
        self.xml_file = xml_file
        self.file = path.basename(xml_file)
        root = ET.parse(xml_file).getroot()
        self.name = root.get("name", self.file)
        self.command = root.get("command", "")
        self.doc = read_doc(root)
        self.result = {"file": self.file, "name": self.name, "command": self.command}

    def kind(self):
        """'external', 'missing' or 'internal' (a program in the paparazzi tree)."""
        if self.command.startswith("$") or not self.command:
            return "external"
        exe = self.command.split()[0]
        if not path.normpath(exe).startswith("sw" + os.sep):
            return "external"
        if not path.isfile(path.join(PPRZ_SRC, exe)):
            return "missing"
        return "internal"

    def run_help(self, timeout):
        cmd = [path.join(PPRZ_SRC, self.command.split()[0])] + self.command.split()[1:] + ["--help"]
        env = dict(os.environ, PAPARAZZI_HOME=PPRZ_HOME, PAPARAZZI_SRC=PPRZ_SRC,
                   IVY_BUS=IVY_BUS, IVYBUS=IVY_BUS, QT_QPA_PLATFORM="offscreen")
        timed_out = False
        try:
            p = subprocess.Popen(cmd, cwd=PPRZ_HOME, env=env, stdin=subprocess.DEVNULL,
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                 start_new_session=True, text=True, errors="replace")
        except OSError as e:
            return -1, "", str(e), False
        try:
            out, err = p.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
            out, err = "", ""
            for sig in (signal.SIGTERM, signal.SIGKILL):
                try:
                    os.killpg(p.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    out, err = p.communicate(timeout=3)
                    break
                except subprocess.TimeoutExpired:
                    continue
        return p.returncode, out or "", err or "", timed_out

    def test(self, timeout, update):
        """Return (ok, skip reason, comment lines)."""
        kind = self.kind()
        self.result["kind"] = kind
        if kind == "external":
            self.result["status"] = "skipped"
            return True, "external program", []
        if kind == "missing":
            self.result["status"] = "failed"
            self.result["reason"] = "command not found (not built?)"
            return False, None, [self.result["reason"]]

        returncode, out, err, timed_out = self.run_help(timeout)
        # programs that print their argv[0]: keep the name only, not the local path
        exe = path.join(PPRZ_SRC, self.command.split()[0])
        out, err = [t.replace(exe, path.basename(exe)).replace(PPRZ_HOME, "$PAPARAZZI_HOME")
                    for t in (out, err)]
        parsed = parse_help(out if out.strip() else err)
        self.result.update(returncode=returncode, timed_out=timed_out, stdout=out, stderr=err, **parsed)
        lines = ["command: %s" % self.command]
        if timed_out or returncode < 0 or not (parsed["options"] or parsed["usage"]):
            reason = failure_reason(out + "\n" + err, returncode, timed_out)
            self.result["reason"] = reason
            tail = ["  | " + l for l in (out + "\n" + err).splitlines() if l.strip()][-8:]
            if reason.startswith("ModuleNotFoundError"):
                self.result["status"] = "skipped"
                return True, "missing Python module: " + reason.split("named ")[-1], tail
            self.result["status"] = "failed"
            return False, None, ["FAILED: %s" % reason] + tail

        if returncode != 0:
            lines.append("note: help printed, exit code %d" % returncode)
        if parsed["usage"]:
            lines.append("usage: %s" % parsed["usage"])
        lines += ["  " + l for l in parsed["description"].split("\n") if parsed["description"]]
        for name, desc in parsed["positionals"]:
            lines.append("  argument %-28s %s" % (name, desc))
        for flags, desc in parsed["options"]:
            lines.append("  option   %-28s %s" % (flags, desc))

        doc = self.doc or {"description": None, "usage": "", "options": []}
        up_to_date = doc["usage"] == parsed["usage"] and doc["options"] == parsed["options"]
        if update and not up_to_date:
            description = doc["description"] if doc["description"] else parsed["description"]
            write_doc(self.xml_file, doc_block(description, parsed["usage"], parsed["options"]))
            lines.append("<doc> updated in %s" % self.file)
        elif not up_to_date:
            self.result["status"] = "outdated"
            self.result["reason"] = "<doc> of %s does not match --help (run %s --update)" % (
                self.file, path.relpath(__file__, PPRZ_HOME))
            return False, None, lines + ["FAILED: " + self.result["reason"]]
        self.result["status"] = "ok"
        return True, None, lines


def main():
    parser = argparse.ArgumentParser(description="Start every Paparazzi Center tool (conf/tools/*.xml) "
                                                 "with --help, list its description and options, and "
                                                 "check the <doc> block of its xml file.")
    parser.add_argument("files", nargs="*", help="tool xml files (default: all of conf/tools)")
    parser.add_argument("-u", "--update", action="store_true",
                        help="write the usage and options of each tool into the <doc> block of its xml file")
    parser.add_argument("-t", "--timeout", type=float, default=10.,
                        help="seconds before a tool that keeps running is stopped (default 10)")
    parser.add_argument("-o", "--output", default=path.join(PPRZ_HOME, "var", "tools_help.json"),
                        help="JSON file with the results (default var/tools_help.json)")
    args = parser.parse_args()

    files = args.files or sorted(path.join(TOOLS_DIR, f) for f in os.listdir(TOOLS_DIR) if f.endswith(".xml"))
    tools = [Tool(f) for f in files]

    ok = TAP.Builder.create(len(tools)).ok
    for tool in tools:
        passed, skip, lines = tool.test(args.timeout, args.update)
        ok(passed, "%s (%s)" % (tool.name, tool.file), skip=skip)
        for line in lines:
            print("# " + line)

    count = {}
    for t in tools:
        count[t.result["status"]] = count.get(t.result["status"], 0) + 1
    print("# ---")
    print("# %d tools: %s" % (len(tools), ", ".join("%d %s" % (n, s) for s, n in sorted(count.items()))))
    for t in tools:
        if t.result["status"] in ("failed", "outdated") or t.result.get("reason"):
            print("# %-8s %-32s %s" % (t.result["status"], t.name, t.result["reason"]))

    os.makedirs(path.dirname(args.output), exist_ok=True)
    with open(args.output, "w") as f:
        json.dump([t.result for t in tools], f, indent=2)
    print("# results saved in %s" % args.output)


if __name__ == "__main__":
    main()
