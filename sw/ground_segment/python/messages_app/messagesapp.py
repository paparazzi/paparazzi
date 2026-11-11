#!/usr/bin/env python3
#
# Copyright (C) 2026 Paparazzi Team
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

"""
Messages (Python): show the last value of every message on the Ivy bus,
like the original Messages tool (sw/ground_segment/tmtc/messages.ml).

One tab per class and sender (e.g. "telemetry:203"). In each tab, the list on
the left holds the messages received, in alphabetical order, with the seconds
since the last one on a green background while the message keeps arriving.
The fields of the selected message are shown on the right as
"type name (unit): value". Values are written as in the original tool:
enum values as "NAME (3)", a field with an alternative unit as
"value (alt_value alt_unit)", with the same unit conversions (units.xml).

A field button can be dragged onto the Real-Time Plotter or the GCS map.
"""

import argparse
import os
import re
import sys
import threading
import time
import xml.etree.ElementTree as ET
from collections import namedtuple

from PyQt5 import QtCore, QtGui, QtWidgets

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                      '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")  # pprzlink

from pprzlink.ivy import IvyMessagesInterface

REFRESH_MS = 250     # time between two updates of the window
LED_ON_S = 0.5       # the age stays on green this long after a message
HELP_TEXT = "Drag-and-drop field on:\n\t- Real-Time Plotter to plot a curve\n\t- GCS map to display as a papget"

# printf formats that can be applied to an int or a float value (format attribute of a field)
INT_FORMAT = re.compile(r"^%[-+ 0#]*\d*[diouxX]$")
FLOAT_FORMAT = re.compile(r"^%[-+ 0#]*\d*(\.\d+)?[eEfFgG]$")

Field = namedtuple("Field", "name type unit alt_unit coef values fmt")


def ocaml_float(x):
    """A float written like OCaml's string_of_float: 12.0 -> '12.', 12.5 -> '12.5'."""
    s = "%.12g" % x
    return s + "." if s.lstrip("-").isdigit() else s


def load_units():
    """The unit conversions of units.xml: (from, to, auto, coef)."""
    for path in (os.path.join(PPRZ_HOME, "var", "units.xml"),
                 os.path.join(PPRZ_HOME, "sw", "ext", "pprzlink", "message_definitions", "common", "units.xml")):
        try:
            return [(u.get("from"), u.get("to"), u.get("auto"), float(u.get("coef")))
                    for u in ET.parse(path).getroot()]
        except (OSError, ET.ParseError, TypeError, ValueError):
            continue
    return []


UNITS = load_units()


def scale_of_units(unit, alt_unit):
    """Coefficient from unit to alt_unit, as pprzlink's scale_of_units with auto="display":
    the first row of units.xml that matches; rows with an auto attribute only need
    the same target unit. None if there is no conversion."""
    if unit == alt_unit:
        return 1.0
    for frm, to, auto, coef in UNITS:
        if to == alt_unit and (auto or frm == unit):
            return coef
    return None


def load_fields():
    """(class, message) -> list of Field, from var/messages.xml."""
    fields = {}
    for cls in ET.parse(os.path.join(PPRZ_HOME, "var", "messages.xml")).getroot():
        for msg in cls.findall("message"):
            lst = []
            for f in msg.findall("field"):
                unit, alt_unit = f.get("unit", ""), f.get("alt_unit")
                coef = f.get("alt_unit_coef")    # as pprzlink's alt_unit_coef_of_xml
                if coef is None:
                    scale = scale_of_units(unit, alt_unit or "")
                    coef = "1." if scale is None else ocaml_float(scale)
                values = [v for v in f.get("values", "").split("|") if v]
                lst.append(Field(f.get("name"), f.get("type"), unit, alt_unit, coef, values, f.get("format")))
            fields[(cls.get("name"), msg.get("name"))] = lst
    return fields


def scalar_text(value, fmt):
    """One value with the format of the field if it fits, like string_of_value."""
    if isinstance(value, float):
        return fmt % value if fmt and FLOAT_FORMAT.match(fmt) else ocaml_float(value)
    if isinstance(value, int):
        return fmt % value if fmt and INT_FORMAT.match(fmt) else str(value)
    return str(value)


def field_text(value, field):
    """The text of a field value: 'NAME (3)' for an enum, 'value (alt_value alt_unit)'
    when there is an alternative unit, arrays as 'v1,v2,...'."""
    if isinstance(value, (list, tuple)):
        text = [scalar_text(v, field.fmt) for v in value]
        return '"%s"' % "".join(text) if field.type.startswith("char") else ",".join(text)
    if isinstance(value, int) and 0 <= value < len(field.values):
        return "%s (%d)" % (field.values[value], value)
    text = scalar_text(value, field.fmt)
    if field.alt_unit is not None:
        try:
            text = "%s (%f%s)" % (text, float(field.coef) * float(text), field.alt_unit)
        except ValueError:
            pass
    return text


class FieldButton(QtWidgets.QPushButton):
    """'type name (unit): ' of a field; can be dragged onto the Real-Time Plotter or the GCS map."""

    def __init__(self, field, drag_text, value_label, window):
        super().__init__("%s %s %s: " % (field.type, field.name, "(%s)" % field.unit if field.unit else ""))
        self.drag_text = drag_text    # "sender:class:message:field:scale"
        self.is_array = '[' in field.type
        self.value_label = value_label
        self.window = window
        self.press_pos = None
        self.setToolTip(HELP_TEXT)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.press_pos = event.pos()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.press_pos is None or \
                (event.pos() - self.press_pos).manhattanLength() < QtWidgets.QApplication.startDragDistance():
            return
        self.press_pos = None
        self.setDown(False)
        text = self.drag_text
        if self.is_array:
            # arrays: ask which element(s) to plot, the whole array by default
            nb = len(self.value_label.text().split(" ")[0].split(","))
            index, ok = QtWidgets.QInputDialog.getText(self, "Index of value to drag", "Index or range in the array ?",
                                                       text="0-%d" % (nb - 1) if nb > 1 else "0")
            if not ok:
                return
            sender, cls, msg, field, scale = text.split(":")
            text = "%s:%s:%s:%s[%s]:%s" % (sender, cls, msg, field, index, scale)
        mime = QtCore.QMimeData()
        mime.setText(text)
        drag = QtGui.QDrag(self)
        drag.setMimeData(mime)
        self.window.show_help(True)     # as the original tool: only the help text during the drag
        drag.exec_(QtCore.Qt.CopyAction)
        self.window.show_help(False)


class MessagePage(QtWidgets.QWidget):
    """The fields of one message: a button 'type name (unit): ' and the value."""

    def __init__(self, sender, msg, fields, window):
        super().__init__()
        self.fields = fields
        self.values = []
        vbox = QtWidgets.QVBoxLayout(self)
        vbox.setSpacing(0)
        for f in fields:
            value = QtWidgets.QLabel("XXXX")
            value.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            drag_text = "%s:%s:%s:%s:%s" % (sender, msg.msg_class, msg.name, f.name, f.coef)
            row = QtWidgets.QHBoxLayout()
            row.addWidget(FieldButton(f, drag_text, value, window))
            row.addWidget(value)
            row.addStretch(1)
            vbox.addLayout(row)
            self.values.append(value)
        vbox.addStretch(1)

    def show_message(self, msg):
        """Write the field values of the message in the labels."""
        for i, (label, field) in enumerate(zip(self.values, self.fields)):
            text = field_text(msg.get_field(i), field)
            if label.text() != text:
                label.setText(text)


class SenderPage(QtWidgets.QWidget):
    """One tab: the list of messages of one sender and the fields of the selected one."""

    def __init__(self, sender, window):
        super().__init__()
        self.sender = sender          # "*" when the tab shows every sender
        self.window = window
        self.items = {}               # message name -> list item
        self.pages = {}               # message name -> MessagePage
        self.list = QtWidgets.QTreeWidget()
        self.list.setColumnCount(2)
        self.list.setHeaderHidden(True)
        self.list.setRootIsDecorated(False)
        self.list.setIndentation(0)
        self.list.setUniformRowHeights(True)
        self.list.header().setStretchLastSection(False)
        self.list.header().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        self.list.header().setSectionResizeMode(1, QtWidgets.QHeaderView.Fixed)
        self.list.header().resizeSection(1, 40)
        self.list.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Expanding)
        self.stack = QtWidgets.QStackedWidget()
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self.stack)
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.list)
        layout.addWidget(scroll, 1)
        self.list.currentItemChanged.connect(self.select)

    def select(self, item, _previous):
        """Show the fields of the message selected in the list."""
        if item is not None:
            self.stack.setCurrentWidget(self.pages[item.text(0)])

    def refresh_page(self, messages, now):
        """Add the new messages, refresh the ages and the shown fields."""
        for name, (t, msg) in messages.items():
            if name not in self.items:
                fields = self.window.fields.get((msg.msg_class, name), [])
                page = MessagePage(self.sender, msg, fields, self.window)
                self.pages[name] = page
                self.stack.addWidget(page)
                item = QtWidgets.QTreeWidgetItem([name, ""])
                item.setTextAlignment(0, QtCore.Qt.AlignCenter)
                item.setTextAlignment(1, QtCore.Qt.AlignCenter)
                self.items[name] = item
                # alphabetical order, like the original tool
                self.list.insertTopLevelItem(sorted(self.items).index(name), item)
                self.list.setFixedWidth(self.list.sizeHintForColumn(0) + 40 + 2 * self.list.frameWidth() + 20)
                if self.list.currentItem() is None:
                    self.list.setCurrentItem(item)
            item = self.items[name]
            age = now - t
            item.setBackground(1, QtGui.QColor("#00ff00") if age < LED_ON_S else QtGui.QBrush())
            age_text = "%d" % age
            if item.text(1) != age_text:
                item.setText(1, age_text)
        current = self.list.currentItem()
        if current is not None:
            name = current.text(0)
            self.pages[name].show_message(messages[name][1])


class MessagesWindow(QtWidgets.QWidget):
    """Main window: one tab per class and sender."""

    def __init__(self, classes, ivy_bus=None, force=False):
        super().__init__()
        self.setWindowTitle("Messages")
        self.setWindowIcon(QtGui.QIcon(os.path.join(PPRZ_HOME, "data", "pictures", "penguin_icon_msg.png")))
        self.resize(660, 460)
        self.fields = load_fields()
        self.force = force
        # (tab name, class, sender) for each -c option; sender None = all senders in one tab
        self.classes = []
        for c in classes:
            cls, _, sender = c.partition(":")
            self.classes.append((c if not sender else cls, cls, sender or None))
        self.alive = set()            # telemetry senders seen with an ALIVE message
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setUsesScrollButtons(True)
        self.help = QtWidgets.QLabel(HELP_TEXT)
        self.help.hide()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.addWidget(self.tabs)
        layout.addWidget(self.help)
        self.senders = {}             # tab name -> SenderPage

        # last message of each tab and name, filled by the Ivy thread
        self.lock = threading.Lock()
        self.latest = {}              # tab name -> (sender, {message name: (time, message)})

        if ivy_bus is not None:
            self.interface = IvyMessagesInterface("Messages (Python)", ivy_bus=ivy_bus)
        else:
            self.interface = IvyMessagesInterface("Messages (Python)")
        self.interface.subscribe(self.message_recv)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(REFRESH_MS)

    def message_recv(self, sender_id, msg):
        """Ivy thread: only keep the last message, the window is refreshed by the timer."""
        sender_id = str(sender_id)
        for name, cls, sender in self.classes:
            if msg.msg_class != cls:
                continue
            if sender == "*":
                # as the original tool: a telemetry sender gets a tab once it sent ALIVE (unless -force)
                if cls == "telemetry" and not self.force and sender_id not in self.alive:
                    if msg.name != "ALIVE":
                        continue
                    self.alive.add(sender_id)
                name, sender = "%s:%s" % (cls, sender_id), sender_id
            elif sender is not None:
                if sender != sender_id:
                    continue
                name = "%s:%s" % (cls, sender)
            with self.lock:
                self.latest.setdefault(name, (sender or "*", {}))[1][msg.name] = (time.time(), msg)

    def refresh(self):
        """GUI thread: new tabs for new senders, then update the shown tab."""
        with self.lock:
            latest = {n: (s, dict(m)) for n, (s, m) in self.latest.items()}
        now = time.time()
        for name, (sender, _) in latest.items():
            if name not in self.senders:
                self.senders[name] = SenderPage(sender, self)
                self.tabs.addTab(self.senders[name], name)
        page = self.tabs.currentWidget()
        if page is not None:
            page.refresh_page(latest[self.tabs.tabText(self.tabs.currentIndex())][1], now)

    def show_help(self, on):
        """During a drag, show the help text instead of the tabs."""
        self.tabs.setVisible(not on)
        self.help.setVisible(on)

    def closeEvent(self, event):
        """Stop Ivy, otherwise it keeps the program running."""
        self.timer.stop()
        self.interface.shutdown()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(description="Show the last value of every message on the Ivy bus")
    parser.add_argument("-b", "--ivy_bus", dest="ivy_bus", help="Ivy bus address and port [default to system IVY bus]")
    parser.add_argument("-c", "--class", dest="classes", action="append", default=[],
                        help="class[:sender] to show, can be repeated (telemetry:* is always shown, "
                             "sender * = one tab per sender)")
    parser.add_argument("-force", "--force", action="store_true",
                        help="show every telemetry sender, not only those that sent ALIVE")
    parser.add_argument("-timestamp", "--timestamp", action="store_true", help="ignored, for compatibility")
    parser.add_argument("-g", "--geometry", help="window geometry, e.g. 500x500+100+100")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    window = MessagesWindow(["telemetry:*"] + args.classes, args.ivy_bus, args.force)
    geometry = re.match(r"^(\d+)x(\d+)(?:([+-]\d+)([+-]\d+))?$", args.geometry or "")
    if geometry:
        window.resize(int(geometry.group(1)), int(geometry.group(2)))
        if geometry.group(3):
            window.move(int(geometry.group(3)), int(geometry.group(4)))
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
