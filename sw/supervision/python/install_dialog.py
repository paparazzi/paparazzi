# Copyright (C) 2026 The Paparazzi Team
# released under GNU GPLv2 or later. See COPYING file.
"""Install components dialog (menu File): the components of the installer with their state, and an Install button.

The list comes from ./install.sh --list --json, the installation is ./install.sh --yes --askpass <components>: its
output goes to the console of the Configuration tab and sudo asks the password in a window
(sw/tools/install/askpass.py).
"""
import json
import os
import subprocess
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
import utils

INSTALL_SH = os.path.join(utils.PAPARAZZI_SRC, "install.sh")
ASKPASS = os.path.join(utils.PAPARAZZI_SRC, "sw", "tools", "install", "askpass.py")
PY_MODES = {
    "system": "apt packages for the system Python",
    "venv": "the venv pprzEnv with the system packages, plus pip for the rest",
    "conda": "pip in the active conda environment",
}


class InstallDialog(QDialog):

    def __init__(self, launch, parent=None):
        """launch(name, cmd, callback) runs a program with its output in the console."""
        QDialog.__init__(self, parent=parent)
        self.launch = launch
        self.setWindowTitle("Install components")
        self.resize(900, 600)
        layout = QVBoxLayout(self)
        self.info_label = QLabel()
        self.info_label.setWordWrap(True)
        layout.addWidget(self.info_label)
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Component", "State", "Description"])
        self.tree.setRootIsDecorated(False)
        layout.addWidget(self.tree)
        options = QHBoxLayout()
        options.addWidget(QLabel("Python packages:"))
        self.python_combo = QComboBox()
        options.addWidget(self.python_combo, 1)
        self.dry_run_checkbox = QCheckBox("Only show the commands")
        options.addWidget(self.dry_run_checkbox)
        layout.addLayout(options)
        buttons = QDialogButtonBox(QDialogButtonBox.Close)
        self.refresh_button = buttons.addButton("Refresh", QDialogButtonBox.ResetRole)
        self.install_button = buttons.addButton("Install", QDialogButtonBox.ApplyRole)
        self.refresh_button.clicked.connect(self.refresh)
        self.install_button.clicked.connect(self.install)
        buttons.rejected.connect(self.close)
        layout.addWidget(buttons)
        self.refresh()

    def refresh(self):
        """Read the components and their state from the installer (./install.sh --list --json)."""
        try:
            out = subprocess.run(["bash", INSTALL_SH, "--list", "--json"], stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, text=True, check=True).stdout
            facts = json.loads(out)
        except (OSError, ValueError, subprocess.CalledProcessError) as e:
            self.info_label.setText("{} --list --json failed: {}".format(INSTALL_SH, e))
            self.install_button.setEnabled(False)
            return
        self.info_label.setText("Components of Paparazzi on {}. Check the ones to install (installed ones are updated "
                                "when checked again). The output is shown in the console of the Configuration tab. {}"
                                .format(facts["system"], facts["warning"]))
        self.tree.clear()
        for comp in facts["components"]:
            state = {"n/a": "not available", "installed": "installed"}.get(comp["state"], "")
            item = QTreeWidgetItem([comp["id"], state, comp["title"]])
            item.setToolTip(2, comp["about"])
            item.setToolTip(1, comp["reason"] or comp["state"])
            if comp["reason"]:
                item.setFlags(item.flags() & ~Qt.ItemIsEnabled)
            else:
                item.setCheckState(0, Qt.Unchecked)
            self.tree.addTopLevelItem(item)
        for col in range(2):
            self.tree.resizeColumnToContents(col)
        self.python_combo.clear()
        for mode, text in PY_MODES.items():
            self.python_combo.addItem("{}: {}".format(mode, text), mode)
        self.python_combo.setCurrentIndex(max(0, self.python_combo.findData(facts["python"])))

    def checked_components(self):
        items = [self.tree.topLevelItem(i) for i in range(self.tree.topLevelItemCount())]
        return [item.text(0) for item in items if item.checkState(0) == Qt.Checked]

    def install(self):
        ids = self.checked_components()
        if not ids:
            QMessageBox.information(self, "Install components", "Check the components to install first.")
            return
        os.environ["SUDO_ASKPASS"] = ASKPASS
        cmd = [INSTALL_SH, "--yes", "--askpass", "--python=" + self.python_combo.currentData()]
        if self.dry_run_checkbox.isChecked():
            cmd.append("--dry-run")
        self.install_button.setEnabled(False)
        self.launch("Install", cmd + ids, self.handle_finished)

    def handle_finished(self, exit_code, _exit_status):
        self.install_button.setEnabled(True)
        self.refresh()
