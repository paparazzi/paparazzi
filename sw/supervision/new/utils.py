
import os
import subprocess
from PyQt5.QtWidgets import *

PAPARAZZI_SRC = os.getenv("PAPARAZZI_HOME")
PAPARAZZI_HOME = os.getenv("PAPARAZZI_HOME", PAPARAZZI_SRC)
CONF_DIR = os.path.join(PAPARAZZI_HOME, "conf/")


# TODO: make it work with shell program such as vim.
def edit_file(file_path, prefix=CONF_DIR):
    path = prefix + file_path
    editor = os.getenv("EDITOR")
    if editor is None:
        editor = "gedit"
    try:
        subprocess.Popen([editor, path])
    except Exception as e:
        print(e)


def make_line(parent: QWidget = None, vertical=False) -> QWidget:
    line = QFrame(parent)
    if vertical:
        line.setFrameShape(QFrame.VLine)
    else:
        line.setFrameShape(QFrame.HLine)
    line.setFrameShadow(QFrame.Sunken)
    return line


def get_version() -> str:
    run_version_exe = os.path.join(PAPARAZZI_HOME, "paparazzi_version")
    return os.popen(run_version_exe).readline().strip()


def get_build_version() -> str:
    bv_path = os.path.join(PAPARAZZI_HOME, "var", "build_version.txt")
    with open(bv_path, 'r') as f:
        version = f.readline().strip()
    return version


def open_terminal(wd, command=None):
    # TODO open konsole, or other terminal emulator if needed.
    cmd = ""
    if command is not None:
        cmd = " -- {}".format(command)
    os.system("gnome-terminal --working-directory {}{}".format(wd, cmd))
