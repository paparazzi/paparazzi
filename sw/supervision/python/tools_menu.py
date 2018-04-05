from PyQt5 import QtCore, QtGui, QtWidgets

ICON_SIZE = (60, 60)
POPUP_SIZE = (1000, 400)
POPUP_GRID_WIDTH = 4
SC_MINIMUM_SIZE = (200, 130)
ICON_NUMBER = 6

class ToolList(QtWidgets.QScrollArea):

    def __init__(self):
        super(ToolList, self).__init__()
        self.frame = QtWidgets.QFrame()
        self.gridLayout = QtWidgets.QGridLayout()
        self.frame.setLayout(self.gridLayout)
        self.setWidget(self.frame)
        self.setWidgetResizable(True)

    def add(self, button):
        count = self.gridLayout.count()
        row = count // POPUP_GRID_WIDTH
        col = count % POPUP_GRID_WIDTH
        self.gridLayout.addWidget(button, row, col)

    def focusOutEvent(self, QFocusEvent):
        self.close()


class ToolsMenu(QtWidgets.QFrame):
    def __init__(self, parent):
        super(ToolsMenu, self).__init__(parent)
        self.hlayout = QtWidgets.QHBoxLayout(self)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        self.toolsFrame = QtWidgets.QFrame()
        self.toolsLayout = QtWidgets.QHBoxLayout(self.toolsFrame)
        self.scrollArea = QtWidgets.QScrollArea()
        self.scrollArea.setWidget(self.toolsFrame)
        self.hlayout.addWidget(self.scrollArea)
        self.scrollArea.setMinimumSize(*SC_MINIMUM_SIZE)
        self.setMinimumSize(SC_MINIMUM_SIZE[0] + 10, SC_MINIMUM_SIZE[1] + 100)

        self.toolsLayout.setSizeConstraint(QtWidgets.QLayout.SetMinimumSize)

        self.scrollArea.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        self.scrollArea.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.scrollArea.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        self.button_more = None
        self.popup = ToolList()
        self.popup.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        self.popup.setMinimumSize(*POPUP_SIZE)
        self.buttons = []
        self.toolsLayout.addStretch(0)

        self._add_more_button()

    def _add_more_button(self):
        if self.button_more is None:
            self.button_more = QtWidgets.QToolButton()
            more_icon = QtGui.QIcon("ui/icons/tools_more.svg")
            self.button_more.setIcon(more_icon)
            self.button_more.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Expanding)
            self.hlayout.addWidget(self.button_more, QtCore.Qt.AlignRight)
            self.button_more.clicked.connect(self.open_popup)

    def add_item(self, name, image_path, callback):
        button = QtWidgets.QToolButton()
        button.setText(name)
        button.setIcon(QtGui.QIcon(image_path))
        button.setIconSize(QtCore.QSize(*ICON_SIZE))
        button.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        button.clicked.connect(callback)

        self.buttons.append(button)

        if self.toolsLayout.count() <= ICON_NUMBER:
            self.toolsLayout.insertWidget(self.toolsLayout.count() - 1, button, QtCore.Qt.AlignLeft)
        else:
            self.popup.add(button)
            button.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            button.show()
            self.popup.adjustSize()

    def open_popup(self):
        bottomLeft = self.mapToGlobal(self.hlayout.geometry().bottomLeft())
        self.popup.move(bottomLeft)
        self.popup.show()
        self.popup.setFocus(QtCore.Qt.PopupFocusReason)
