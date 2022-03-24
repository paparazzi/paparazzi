
import conf
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui
from main_panel import MainPanel
import utils


class PprzCenter(QMainWindow):
    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent=parent)
        self.tabwidget = QTabWidget(parent=self)
        self.setCentralWidget(self.tabwidget)
        self.main_panel = MainPanel()
        self.tabwidget.addTab(self.main_panel, "test")
        self.status_msg = QLabel()
        self.statusBar().addWidget(self.status_msg)
        self.fill_status_bar()
        self.statusBar().show()
        self.main_panel.msg_error.connect(self.handle_error)
        self.main_panel.clear_error.connect(self.clear_error)

    def quit(self):
        pass
        # TODO ask to save conf if it has been edited.
        #self.main_panel.conf.save()

    def fill_status_bar(self):
        home_widget = QWidget()
        home_lay = QHBoxLayout(home_widget)
        home_lay.addWidget(QLabel("HOME: ", home_widget))
        home_button = QPushButton(utils.PAPARAZZI_HOME, home_widget)
        home_lay.addWidget(home_button)
        home_button.clicked.connect(lambda: utils.open_terminal(utils.PAPARAZZI_HOME))
        self.statusBar().addPermanentWidget(home_widget)
        self.statusBar().addPermanentWidget(utils.make_line(None, True))
        label_version = QLabel("Version={}".format(utils.get_version()))
        self.statusBar().addPermanentWidget(label_version)
        self.statusBar().addPermanentWidget(utils.make_line(None, True))
        label_build = QLabel("Build={}".format(utils.get_build_version()))
        self.statusBar().addPermanentWidget(label_build)

    def handle_error(self, msg):
        self.status_msg.setText(msg)
        self.statusBar().setStyleSheet("background-color: red;")
        # self.statusBar().showMessage(msg)

    def clear_error(self):
        self.status_msg.setText("")
        self.statusBar().setStyleSheet("")


if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    main_window = PprzCenter()
    main_window.show()
    qApp.aboutToQuit.connect(main_window.quit)
    sys.exit(app.exec_())

