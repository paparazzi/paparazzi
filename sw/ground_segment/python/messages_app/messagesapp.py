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
laid out like the original Messages tool (sw/ground_segment/tmtc/messages.ml).

One tab per sender (e.g. "telemetry:203"). In each tab, the list on the left
holds the messages received from that sender, in alphabetical order, with a
green LED while the message keeps arriving and the seconds since the last one.
The fields of the selected message are shown on the right as
"type name (unit): value", with the names of enum values and the value in the
alternative unit (e.g. degrees) when the message definition has one.

A field name can be dragged onto the Real-Time Plotter to plot it, as in the
original tool.
"""

import argparse
import os
import xml.etree.ElementTree as ET
import sys
import threading
import time

from PyQt5 import QtCore, QtGui, QtWidgets

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                      '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")  # pprzlink

from pprzlink.ivy import IvyMessagesInterface
from pprzlink import messages_xml_map

REFRESH_MS = 250     # time between two updates of the window
LED_ON_S = 0.5       # the LED stays green this long after a message


def load_unit_coefs():
    """Conversion coefficients between units (e.g. rad -> deg), from units.xml of pprzlink."""
    coefs = {}
    for path in (os.path.join(PPRZ_HOME, "var", "units.xml"),
                 os.path.join(PPRZ_HOME, "sw", "ext", "pprzlink", "message_definitions", "common", "units.xml")):
        try:
            for unit in ET.parse(path).getroot():
                coefs[(unit.get("from"), unit.get("to"))] = float(unit.get("coef"))
            break
        except (OSError, ET.ParseError, TypeError, ValueError):
            continue
    return coefs


UNIT_COEFS = load_unit_coefs()


def alt_coef(unit, alt_unit, coef):
    """Coefficient from unit to alt_unit: the alt_unit_coef of the message definition,
    otherwise the one of units.xml (pprzlink gives 1 when alt_unit_coef is not set)."""
    if not alt_unit or not unit or unit == alt_unit:
        return coef
    if coef and coef != 1.0:
        return coef
    return UNIT_COEFS.get((unit, alt_unit), coef)


def led_icon(color):
    """A small round LED icon of the given color."""
    pixmap = QtGui.QPixmap(12, 12)
    pixmap.fill(QtCore.Qt.transparent)
    painter = QtGui.QPainter(pixmap)
    painter.setRenderHint(QtGui.QPainter.Antialiasing)
    painter.setBrush(QtGui.QColor(color))
    painter.setPen(QtGui.QColor("#555555"))
    painter.drawEllipse(1, 1, 10, 10)
    painter.end()
    return QtGui.QIcon(pixmap)


def field_text(value, enum_names, fmt, alt_coef, alt_unit):
    """The text of one field value, like the original tool: 'NAME (3)' for enums,
    'value (alt_value alt_unit)' when there is an alternative unit."""
    if isinstance(value, (list, tuple)):
        return ",".join(str(v) for v in value)
    if enum_names:
        try:
            return "%s (%d)" % (enum_names[int(value)], int(value))
        except (ValueError, IndexError):
            pass
    text = str(value)
    if fmt:
        try:
            text = fmt % value
        except (TypeError, ValueError):
            pass
    if alt_coef and alt_unit:
        try:
            text = "%s (%g%s)" % (text, float(value) * alt_coef, alt_unit)
        except (TypeError, ValueError):
            pass
    return text


class FieldLabel(QtWidgets.QLabel):
    """Name of a field; can be dragged onto the Real-Time Plotter."""

    def __init__(self, text, drag_text, is_array):
        super().__init__(text)
        self.drag_text = drag_text    # "sender:class:message:field:scale"
        self.is_array = is_array
        self.press_pos = None
        self.setToolTip("Drag onto the Real-Time Plotter to plot this field")
        self.setCursor(QtCore.Qt.OpenHandCursor)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.press_pos = event.pos()

    def mouseMoveEvent(self, event):
        if self.press_pos is None or \
                (event.pos() - self.press_pos).manhattanLength() < QtWidgets.QApplication.startDragDistance():
            return
        self.press_pos = None
        text = self.drag_text
        if self.is_array:
            # arrays: ask which element to plot, as the original tool
            index, ok = QtWidgets.QInputDialog.getText(self, "Index of value to drag",
                                                       "Index or range in the array?", text="0")
            if not ok:
                return
            sender, cls, msg, field, scale = text.split(":")
            text = "%s:%s:%s:%s[%s]:%s" % (sender, cls, msg, field, index, scale)
        mime = QtCore.QMimeData()
        mime.setText(text)
        drag = QtGui.QDrag(self)
        drag.setMimeData(mime)
        drag.exec_(QtCore.Qt.CopyAction)


class MessagePage(QtWidgets.QWidget):
    """The fields of one message: 'type name (unit):' and the value."""

    def __init__(self, sender, msg):
        super().__init__()
        self.values = []
        self.enums = messages_xml_map.get_msg_values_enum(msg.msg_class, msg.msg_id)
        self.formats = msg.fieldformats
        self.alt_units = msg.fieldalt_units
        self.coefs = [alt_coef(u, au, c) for u, au, c in zip(msg.fieldunits, self.alt_units, msg.fieldcoefs)]
        grid = QtWidgets.QGridLayout(self)
        grid.setVerticalSpacing(2)
        for i, (name, ftype, unit) in enumerate(zip(msg.fieldnames, msg.fieldtypes, msg.fieldunits)):
            label = "%s %s%s:" % (ftype, name, " (%s)" % unit if unit else "")
            scale = self.coefs[i] if self.coefs[i] else 1.0
            drag_text = "%s:%s:%s:%s:%g" % (sender, msg.msg_class, msg.name, name, scale)
            grid.addWidget(FieldLabel(label, drag_text, '[' in ftype), i, 0)
            value = QtWidgets.QLabel("XXXX")
            value.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            grid.addWidget(value, i, 1)
            self.values.append(value)
        grid.setColumnStretch(1, 1)
        grid.setRowStretch(len(self.values), 1)

    def show_message(self, msg):
        """Write the field values of the message in the labels."""
        for i, label in enumerate(self.values):
            enum_names = self.enums[i] if i < len(self.enums) else None
            text = field_text(msg.get_field(i), enum_names, self.formats[i], self.coefs[i], self.alt_units[i])
            if label.text() != text:
                label.setText(text)


class SenderPage(QtWidgets.QSplitter):
    """One tab: the list of messages of one sender and the fields of the selected one."""

    def __init__(self, sender):
        super().__init__(QtCore.Qt.Horizontal)
        self.sender = sender
        self.items = {}       # message name -> list item
        self.pages = {}       # message name -> MessagePage
        self.list = QtWidgets.QTreeWidget()
        self.list.setHeaderLabels(["Message", "s"])
        self.list.setRootIsDecorated(False)
        self.list.setIndentation(0)
        self.list.setUniformRowHeights(True)
        self.list.header().setStretchLastSection(False)
        self.list.header().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        self.list.header().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        self.stack = QtWidgets.QStackedWidget()
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self.stack)
        self.addWidget(self.list)
        self.addWidget(scroll)
        self.setStretchFactor(1, 1)
        self.setSizes([200, 300])
        self.list.currentItemChanged.connect(self.select)

    def select(self, item, _previous):
        """Show the fields of the message selected in the list."""
        if item is not None:
            self.stack.setCurrentWidget(self.pages[item.text(0)])

    def refresh_page(self, messages, now, led_on, led_off):
        """Add the new messages, refresh the LEDs and ages and the shown fields."""
        for name, (t, msg) in messages.items():
            if name not in self.items:
                page = MessagePage(self.sender, msg)
                self.pages[name] = page
                self.stack.addWidget(page)
                item = QtWidgets.QTreeWidgetItem([name, ""])
                item.setTextAlignment(1, QtCore.Qt.AlignRight)
                self.items[name] = item
                # wide enough for the longest message name
                width = self.list.fontMetrics().horizontalAdvance(name) + 80
                if width > self.list.minimumWidth():
                    self.list.setMinimumWidth(width)
                # alphabetical order, like the original tool
                names = sorted(self.items)
                self.list.insertTopLevelItem(names.index(name), item)
                if self.list.currentItem() is None:
                    self.list.setCurrentItem(item)
            item = self.items[name]
            age = now - t
            item.setIcon(0, led_on if age < LED_ON_S else led_off)
            age_text = "%d" % age
            if item.text(1) != age_text:
                item.setText(1, age_text)
        current = self.list.currentItem()
        if current is not None:
            name = current.text(0)
            self.pages[name].show_message(messages[name][1])


class MessagesWindow(QtWidgets.QWidget):
    """Main window: one tab per sender."""

    def __init__(self, msg_class, ivy_bus=None):
        super().__init__()
        self.setWindowTitle("Messages")
        self.resize(660, 460)
        self.msg_class = msg_class
        self.led_on = led_icon("#00e000")
        self.led_off = led_icon("#c8c8c8")
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setUsesScrollButtons(True)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.addWidget(self.tabs)
        self.senders = {}     # sender id -> SenderPage

        # last message of each sender and name, filled by the Ivy thread
        self.lock = threading.Lock()
        self.latest = {}      # sender id -> {message name: (time, message)}

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
        if msg.msg_class != self.msg_class:
            return
        with self.lock:
            self.latest.setdefault(sender_id, {})[msg.name] = (time.time(), msg)

    def refresh(self):
        """GUI thread: new tabs for new senders, then update the shown tab."""
        with self.lock:
            latest = {s: dict(m) for s, m in self.latest.items()}
        now = time.time()
        for sender in sorted(latest, key=str):
            if sender not in self.senders:
                page = SenderPage(sender)
                self.senders[sender] = page
                self.tabs.addTab(page, "%s:%s" % (self.msg_class, sender))
        page = self.tabs.currentWidget()
        if page is not None:
            page.refresh_page(latest[page.sender], now, self.led_on, self.led_off)

    def closeEvent(self, event):
        """Stop Ivy, otherwise it keeps the program running."""
        self.timer.stop()
        self.interface.shutdown()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(description="Show the last value of every message on the Ivy bus")
    parser.add_argument("-b", "--ivy_bus", dest="ivy_bus", help="Ivy bus address and port [default to system IVY bus]")
    parser.add_argument("-c", "--class", dest="msg_class", default="telemetry",
                        help="message class to show [default: telemetry]")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    window = MessagesWindow(args.msg_class, args.ivy_bus)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
