# Copyright (C) 2008-2022 The Paparazzi Team
# released under GNU GPLv2 or later. See COPYING file.
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QUrl, Qt

try:
    from generated.ui_doc_viewer import Ui_DocPanel
except ImportError:
    class Ui_DocPanel:
        def setupUi(self, DocPanel):
            self.deactivated = True
            self.lay = QVBoxLayout(DocPanel)
            label = QLabel("Please install 'python3-pyqt5.qtwebkit' to view the doc.", DocPanel)
            label.setTextInteractionFlags(Qt.TextSelectableByMouse | Qt.TextSelectableByKeyboard)
            self.lay.addWidget(label)


from PyQt5.QtGui import QDesktopServices
import utils
import os
import subprocess
import conf

LOCAL_DOC_ROOT = os.path.join(utils.PAPARAZZI_HOME, "doc/sphinx/build/html/")
INTERNET_DOC_ROOT = "https://paparazzi-uav.readthedocs.io/en/latest/"


class DocPanel(QWidget, Ui_DocPanel):

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.setupUi(self)
        try:
            self.deactivated
            return
        except AttributeError:
            ...
        self.current_ac = None
        self.doc_source_combo.currentTextChanged.connect(self.change_doc_source)
        self.open_browser_button.clicked.connect(lambda: QDesktopServices.openUrl(self.webView.url()))
        self.urlLineEdit.returnPressed.connect(lambda: self.webView.setUrl(QUrl(self.urlLineEdit.text())))
        self.webView.loadFinished.connect(self.load_finished)
        self.webView.loadProgress.connect(self.load_progress)
        self.modules_list.currentTextChanged.connect(self.handle_select_module)
        self.depends_modules_list.currentTextChanged.connect(self.handle_select_module)
        self.unloaded_modules_list.currentTextChanged.connect(self.handle_select_module)
        self.searchLineEdit.textChanged.connect(self.filter_modules)
        self.target_combo.currentTextChanged.connect(self.target_changed)
        self.backButton.clicked.connect(self.webView.back)
        self.webView.urlChanged.connect(lambda u: self.urlLineEdit.setText(u.toString()))
        url = self.make_url("index.html")
        self.webView.load(url)

    def load_finished(self, finished):
        # print(f"load finished: {finished}")
        ...
        # if not finished:
        #     self.webView.setHtml("<h1>Doc not found!</h1>"
        #                          "<p>Documentation not found! Please build it first.</p>")

    def load_progress(self, progress):
        # print(f"load progress: {progress}")
        ...

    def set_aircraft(self, ac: conf.Aircraft):
        try:
            self.deactivated
            return
        except AttributeError:
            ...
        self.current_ac = ac
        targets = ac.boards.keys()
        self.target_combo.clear()
        self.target_combo.addItem("all")
        self.target_combo.addItems(targets)

    def target_changed(self, target):
        self.modules_list.clear()
        self.depends_modules_list.clear()
        self.unloaded_modules_list.clear()
        if target != "":
            modules = self.get_all_modules(self.current_ac, target)
            for module_path, module_type in modules:
                if module_type == "U" or module_type == "_":
                    self.modules_list.addItem(module_path)
                elif module_type == "N":
                    self.unloaded_modules_list.addItem(module_path)
                else:
                    self.depends_modules_list.addItem(module_path)
        self.filter_modules(self.searchLineEdit.text())

    def filter_modules(self, filter_txt):
        def filter_list(list):
            for i in range(list.count()):
                if filter_txt != "":
                    txt = list.item(i).text()
                    list.item(i).setHidden(filter_txt not in txt)
                else:
                    list.item(i).setHidden(False)
        filter_list(self.modules_list)
        filter_list(self.depends_modules_list)
        filter_list(self.unloaded_modules_list)

    def make_url(self, doc):
        if self.doc_source_combo.currentText() == "Local":
            path = os.path.join(LOCAL_DOC_ROOT, doc)
            return QUrl.fromLocalFile(path)
        else:
            path = INTERNET_DOC_ROOT + doc
            return QUrl(path)

    def change_doc_source(self, source):
        current_url = self.webView.url().toString()
        lu = QUrl.fromLocalFile(LOCAL_DOC_ROOT).toString()
        iu = QUrl(INTERNET_DOC_ROOT).toString()
        url = QUrl("http://perdu.com")
        if source == "Internet" and lu in current_url:
            url = QUrl(current_url.replace(lu, iu))
        if source == "Local" and iu in current_url:
            url = QUrl(current_url.replace(iu, lu))
        self.webView.load(url)

    def handle_select_module(self, txt):
        if txt == "":
            return
        url = self.make_url("modules/{}.html".format(txt))
        self.webView.load(url)

    def get_all_modules(self, ac: conf.Aircraft, target: str):
        args = [conf.MOD_DEP, "-ac", ac.name, "-af", ac.airframe, "-fp", ac.flight_plan]
        if target != "all":
            args.extend(["-t", target])
        completed = subprocess.run(args, capture_output=True)
        if completed.returncode != 0:
            return []   # completed.returncode, completed.stderr

        modules_list = []
        module_lines = completed.stdout.decode().strip().split("\n")
        for module_line in module_lines:
            args = module_line.split(" ")
            module_path = args[0]
            module = utils.remove_prefix(args[0], os.path.join(utils.CONF_DIR, "modules/"))
            module = utils.remove_suffix(module, ".xml")
            module_type = args[1] if len(args) > 1 else "_"
            modules_list.append((module, module_type))
        return modules_list
