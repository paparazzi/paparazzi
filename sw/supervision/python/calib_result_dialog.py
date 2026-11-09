from PyQt5 import QtCore, QtGui, QtWidgets


def keep_old_calibration(sensor, label, conf, new, comments):
    """IMU_<sensor>_CALIB define with the new structs ({abi_id: XML}) and, from the airframe file of conf,
    the old struct of every sensor which is not calibrated now, so it can be pasted as is.
    Returns the XML and notes for the summary."""
    import calibration_utils
    if not new:
        return "", []
    old = calibration_utils.airframe_calib_structs(conf.airframe if conf is not None else None, sensor)
    structs, kept = calibration_utils.merge_calib_structs(old, new)
    notes = [f"{label} {abi_id}: calibration of the airframe kept" for abi_id in kept]
    failed = any("not calibrated" in c for c in comments)
    if failed and not old:
        notes.append(f"No IMU_{sensor}_CALIB found in the airframe file: add the previous calibration "
                     "of the sensors not calibrated now by hand")
    xml = calibration_utils.format_xml_define(sensor, structs, comments + notes)
    return xml, notes


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
