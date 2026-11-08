#!/usr/bin/env python3
"""Asks the sudo password in a window: sudo -A runs this program (SUDO_ASKPASS) and reads the password it prints.

Used by the installer window without terminal (ui.py) and by the Paparazzi Center (install.sh --askpass).
zenity is on every Ubuntu desktop; without it the Qt of the Paparazzi Center asks.
"""

import subprocess
import sys

TITLE = 'Paparazzi installer: password for sudo'

try:
    sys.exit(subprocess.run(['zenity', '--password', '--title=' + TITLE], stderr=subprocess.DEVNULL).returncode)
except FileNotFoundError:
    pass
from PyQt5.QtWidgets import QApplication, QInputDialog, QLineEdit  # noqa: E402

app = QApplication(sys.argv[:1])
password, ok = QInputDialog.getText(None, TITLE, 'Password for sudo:', QLineEdit.Password)
if not ok:
    sys.exit(1)
print(password)
