from PyQt5 import QtCore, QtGui, QtWidgets


class CalibResultDialog(QtWidgets.QDialog):
    """Calibration result to paste in the IMU section of the airframe file:
    a text box with the XML and a button to copy it all."""

    def __init__(self, title, summary, xml, parent=None):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.resize(720, 520)
        layout = QtWidgets.QVBoxLayout(self)

        label = QtWidgets.QLabel(summary, self)
        label.setWordWrap(True)
        layout.addWidget(label)

        self.text = QtWidgets.QPlainTextEdit(self)
        self.text.setPlainText(xml)
        self.text.setReadOnly(True)
        self.text.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self.text.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        layout.addWidget(self.text, 1)

        buttons = QtWidgets.QHBoxLayout()
        layout.addLayout(buttons)
        self.copy_status = QtWidgets.QLabel(self)
        buttons.addWidget(self.copy_status, 1)
        copy_button = QtWidgets.QPushButton("Copy to clipboard", self)
        copy_button.setDefault(True)
        copy_button.clicked.connect(self.copy)
        copy_button.setEnabled(bool(xml))
        buttons.addWidget(copy_button)
        close_button = QtWidgets.QPushButton("Close", self)
        close_button.clicked.connect(self.accept)
        buttons.addWidget(close_button)

    def copy(self):
        QtWidgets.QApplication.clipboard().setText(self.text.toPlainText())
        self.copy_status.setText("Copied: paste it in the IMU section of the airframe file, "
                                 "replacing the old calibration define")
