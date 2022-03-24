from PyQt5.QtWidgets import QWidget
from generated.ui_build import Ui_Build


class BuildWidget(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.ui = Ui_Build()
        self.ui.setupUi(self)
