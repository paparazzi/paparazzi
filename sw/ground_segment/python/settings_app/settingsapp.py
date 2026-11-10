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
Settings (Python): change the settings of an aircraft.

Reads the settings of the aircraft (var/aircrafts/<name>/settings.xml, so the
aircraft must be built) and shows them in one tab per settings group. Each
setting has a value field (or a list for settings with named values), a slider
and an Apply button that sends the DL_SETTING message on the Ivy bus.
The values sent by the aircraft (DL_VALUE) and by other ground tools
(DL_SETTING) are shown as they arrive. A bold value is set here and not yet
confirmed by the aircraft.

With several --ac_id, the settings of the first aircraft are shown and sent to
all of them.
"""

import argparse
import os
import sys

from PyQt5 import QtCore, QtGui, QtWidgets

PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                      '../../../..')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")

from settings_tool import IvySettingsInterface


class SettingRow:
    """The widgets of one setting: name, value field, slider and Apply button."""

    def __init__(self, setting, grid, row, send_cb):
        self.setting = setting
        self.send_cb = send_cb
        # the slider counts steps from the minimum value
        self.nb_steps = max(1, int(round((setting.max_value - setting.min_value) / setting.step)))
        # number of decimals shown, from the step (0.1 -> 1, 0.25 -> 2, 1 -> 0)
        self.decimals = len(("%.10f" % setting.step).rstrip('0').split('.')[1])

        self.label = QtWidgets.QLabel(setting.shortname)
        self.label.setToolTip("%s\nindex %d, min %g, max %g, step %g" %
                              (setting.var, setting.index, setting.min_value, setting.max_value, setting.step))

        if setting.values is not None:
            # named values (e.g. modes): a list instead of a number
            self.edit = QtWidgets.QComboBox()
            self.edit.addItems(setting.values)
            self.edit.setCurrentIndex(-1)
            self.edit.activated.connect(self.on_combo)
        else:
            self.edit = QtWidgets.QLineEdit()
            self.edit.setMaximumWidth(120)
            self.edit.textEdited.connect(self.on_text)
            self.edit.returnPressed.connect(self.on_apply)

        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(0, self.nb_steps)
        self.slider.setMinimumWidth(200)
        self.slider.sliderMoved.connect(self.on_slider)
        self.slider.actionTriggered.connect(self.on_slider_action)

        self.button = QtWidgets.QPushButton("Apply")
        self.button.clicked.connect(self.on_apply)

        grid.addWidget(self.label, row, 0)
        grid.addWidget(self.edit, row, 1)
        grid.addWidget(self.slider, row, 2)
        grid.addWidget(self.button, row, 3)

    def snap(self, value):
        """Round a value to the step of the setting and keep it between min and max."""
        s = self.setting
        value = min(max(value, s.min_value), s.max_value)
        return round(s.min_value + round((value - s.min_value) / s.step) * s.step, self.decimals)

    def to_slider(self, value):
        """Slider position of a value."""
        return int(round((value - self.setting.min_value) / self.setting.step))

    def from_slider(self, pos):
        """Value of a slider position."""
        return self.snap(self.setting.min_value + pos * self.setting.step)

    def format(self, value):
        """Text of a value, with the number of decimals of the step."""
        return "%.*f" % (self.decimals, value)

    def value(self):
        """Value to send: the one in the field, or the slider one if the field is not a number."""
        if isinstance(self.edit, QtWidgets.QComboBox):
            if self.edit.currentIndex() < 0:
                return None
            return self.snap(self.setting.min_value + self.edit.currentIndex() * self.setting.step)
        try:
            return self.snap(float(self.edit.text()))
        except ValueError:
            return self.from_slider(self.slider.value())

    def set_bold(self, bold):
        """Bold: value set here, normal: value received from the bus."""
        font = self.edit.font()
        font.setBold(bold)
        self.edit.setFont(font)

    def show_value(self, value):
        """Show a value in the field and the slider (without sending it)."""
        if isinstance(self.edit, QtWidgets.QComboBox):
            idx = int(round((value - self.setting.min_value) / self.setting.step))
            self.edit.setCurrentIndex(idx if 0 <= idx < self.edit.count() else -1)
        else:
            self.edit.setText(self.format(value))
        self.slider.blockSignals(True)
        self.slider.setValue(self.to_slider(value))
        self.slider.blockSignals(False)

    def on_text(self, text):
        """Value typed: move the slider."""
        try:
            value = float(text)
        except ValueError:
            return
        self.slider.blockSignals(True)
        self.slider.setValue(self.to_slider(self.snap(value)))
        self.slider.blockSignals(False)
        self.set_bold(True)

    def on_combo(self, idx):
        """Named value chosen: move the slider."""
        self.show_value(self.setting.min_value + idx * self.setting.step)
        self.set_bold(True)

    def on_slider(self, pos):
        """Slider dragged: copy its value in the field."""
        self.show_value(self.from_slider(pos))
        self.set_bold(True)

    def on_slider_action(self, action):
        """Slider moved by the keyboard, the mouse wheel or a click next to the handle."""
        if action != QtWidgets.QAbstractSlider.SliderMove:
            self.on_slider(self.slider.sliderPosition())

    def on_apply(self):
        """Send the value to the aircraft."""
        value = self.value()
        if value is None:
            return
        self.show_value(value)
        self.set_bold(True)
        self.send_cb(self.setting.index, self.format(value))

    def has_focus(self):
        """True while the user edits this setting."""
        return self.edit.hasFocus() or self.slider.isSliderDown()


class SettingsWindow(QtWidgets.QMainWindow):
    """Main window: one tab per settings group."""

    # Ivy callbacks run in the Ivy thread: the values go to the GUI through these signals
    value_received = QtCore.pyqtSignal(int, str, bool)
    state_changed = QtCore.pyqtSignal(bool, str)

    def __init__(self, ac_ids):
        super().__init__()
        self.settings = IvySettingsInterface(ac_ids)
        self.title = "Settings %s (%s)" % (ac_ids, self.settings.GetACName())
        self.setWindowTitle(self.title)
        self.rows = {}  # SettingRow per setting index

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setUsesScrollButtons(True)
        self.setCentralWidget(self.tabs)
        for group in self.settings.groups:
            if not group.member_list:
                continue  # group that only contains other groups
            page = QtWidgets.QWidget()
            grid = QtWidgets.QGridLayout(page)
            grid.setColumnStretch(2, 1)
            for row, setting in enumerate(group.member_list):
                self.rows[setting.index] = SettingRow(setting, grid, row, self.send_setting)
            grid.setRowStretch(len(group.member_list), 1)
            scroll = QtWidgets.QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setWidget(page)
            self.tabs.addTab(scroll, group.name)

        self.value_received.connect(self.update_value)
        self.state_changed.connect(self.update_state)
        self.settings.RegisterCallback(self.value_received.emit)
        self.settings.RegisterStateCallback(self.state_changed.emit)
        self.resize(640, 400)

    def send_setting(self, index, value):
        """Send a setting with DL_SETTING."""
        self.settings.lookup[index].value = value
        self.settings.SendSetting(index)

    def update_value(self, index, value, from_remote):
        """Show a value received on the bus (DL_VALUE from the aircraft, DL_SETTING from the ground)."""
        row = self.rows.get(index)
        if row is None:
            return
        if from_remote and row.has_focus():
            return  # don't overwrite what the user is editing
        try:
            row.show_value(float(value))
        except ValueError:
            return
        row.set_bold(not from_remote)

    def update_state(self, valid, message):
        """Sending settings blocked (e.g. aircraft running another build) or allowed again."""
        self.setWindowTitle(self.title if valid else "%s - NOT SENDING: %s" % (self.title, message))

    def closeEvent(self, event):
        """Stop Ivy, otherwise it keeps the program running."""
        self.settings.OnClose()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(description="Change the settings of an aircraft")
    parser.add_argument("-a", "--ac_id", type=int, action="append", required=True,
                        help="aircraft ID (several may be given: the settings of the first one are sent to all)")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    window = SettingsWindow(args.ac_id)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
