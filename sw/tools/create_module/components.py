from PyQt5 import QtWidgets
from init_ui import Ui_Init
from periodic_ui import Ui_Periodic
from event_ui import Ui_Event
from datalink_ui import Ui_Datalink
from abi_ui import Ui_Abi

class ComponentWidget(QtWidgets.QFrame):
    def __init__(self, comp_type):
        QtWidgets.QWidget.__init__(self)
        self.comp_type = comp_type
        self.ui = None
        if self.comp_type == "Init":
            self.ui = Ui_Init()
        if self.comp_type == "Event":
            self.ui = Ui_Event()
        if self.comp_type == "Periodic":
            self.ui = Ui_Periodic()
        if self.comp_type == "Datalink":
            self.ui = Ui_Datalink()
        if self.comp_type == "Abi":
            self.ui = Ui_Abi()
        self.ui.setupUi(self)
