from PyQt5.QtWidgets import QWidget
from generated.ui_upload import Ui_Upload


class UploadWidget(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.ui = Ui_Upload()
        self.ui.setupUi(self)
