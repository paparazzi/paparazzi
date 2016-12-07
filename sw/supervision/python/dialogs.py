# Paparazzi center utilities
#
# Copyright (C) 2016 ENAC, Florian BITARD (intern student)
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
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.

###############################################################################
# [Imports]

import ui.set_manager as manager
import ui.popup as popup

import PyQt5.QtWidgets as Widgets
import logging
import os


###############################################################################
# [Constants]

LOGGER = logging.getLogger("[DIALOGS]")

UI_DIR = "ui"

DATA_CHANGED_POPUP_TYPE = "data changed"
DATA_CHANGED_POPUP_HTML = "data_changed.html"
CREDITS_POPUP_TYPE = "credits"
CREDITS_POPUP_HTML = "credits.html"
TUTORIAL_POPUP_TYPE = "tutorial"
TUTORIAL_POPUP_HTML = "tutorial.html"


###############################################################################
# [SettingsManager class]

class SettingsManager(Widgets.QDialog):
    """Class to manage the settings manager HMI."""
    def __init__(self):
        super(SettingsManager, self).__init__()
        self.ui = manager.Ui_Dialog()
        self.ui.setupUi(self)


###############################################################################
# [SettingsManager class]

class Popup(Widgets.QDialog):
    """Class to manage the settings manager HMI."""
    def __init__(self, popup_type=None):
        super(Popup, self).__init__()
        self.ui = popup.Ui_Dialog()
        self.ui.setupUi(self)
        self.ui.textBrowser.setOpenExternalLinks(True)

        self.type = popup_type

        self.set_popup_details()

    def set_popup_details(self):
        """
        -> Set the title, the text and the dialog buttons of the popup window
        according to its type.
        """
        if self.type == DATA_CHANGED_POPUP_TYPE:
            self.setWindowTitle("/!\ Some unsaved data found !")
            self.ui.buttonBox.setStandardButtons(
                Widgets.QDialogButtonBox.Cancel | Widgets.QDialogButtonBox.Save)
            html_file = DATA_CHANGED_POPUP_HTML
        elif self.type == CREDITS_POPUP_TYPE:
            self.setWindowTitle("Paparazzi UAV Center credits "
                                "(Python/Qt version)")
            self.ui.buttonBox.setStandardButtons(Widgets.QDialogButtonBox.Close)
            html_file = CREDITS_POPUP_HTML
        elif self.type == TUTORIAL_POPUP_TYPE:
            self.setWindowTitle("Tutorials and documents")
            self.ui.buttonBox.setStandardButtons(Widgets.QDialogButtonBox.Close)
            html_file = TUTORIAL_POPUP_HTML
        else:
            self.setWindowTitle("Popup without specific type.")
            html_file = "none.html"

        try:
            html_config_path = os.path.join(UI_DIR, html_file)
            with open(html_config_path, 'r') as popup_html_content:
                self.ui.textBrowser.setHtml(popup_html_content.read())
        except FileNotFoundError:
            print("Popup HTML configuration file not found ! "
                  "('%s' should be in 'ui' directory.)" % html_file)
