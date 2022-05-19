# Copyright (C) 2008-2022 The Paparazzi Team
# released under GNU GPLv2 or later. See COPYING file.

from generated.ui_operation_panel import Ui_OperationPanel
from PyQt5.QtWidgets import *


class OperationPanel(QWidget, Ui_OperationPanel):

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.setupUi(self)
        self.session.set_console(self.console)
