#!/usr/bin/env python3

# World environment (time, wind, ...) for multi-AC simulation
#
# Copyright (C) 2024 Mael FEURGARD <mael.feurgard@enac.fr>
# 
# This file is part of paparazzi.
# 
# paparazzi is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with paparazzi.  If not, see <https://www.gnu.org/licenses/>.

import os
import sys
import math

PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage


from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QWidget,QMainWindow,QApplication,\
                            QLabel

from gaia_python.betterHSlider import BetterHSlider
from gaia_python.AngleSelector import AngleSelector

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Gaia")
        Dialog.resize(663, 331)
        
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(Dialog)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        
        self.main_horizontalLayout = QtWidgets.QHBoxLayout()
        self.main_horizontalLayout.setObjectName("main_horizontalLayout")
        
        self.sliders_frame = QtWidgets.QFrame(Dialog)
        self.sliders_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.sliders_frame.setFrameShadow(QtWidgets.QFrame.Plain)
        self.sliders_frame.setObjectName("sliders_frame")
        
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.sliders_frame)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        
        self.sliders_verticalLayout = QtWidgets.QVBoxLayout()
        self.sliders_verticalLayout.setObjectName("sliders_verticalLayout")
        
        self.time_slider = BetterHSlider(self.sliders_frame)
        self.time_slider.setObjectName("time_slider")
        self.time_slider.setText("Time scale")
        self.time_slider.setDecimals(1)
        self.time_slider.setRange(0.1,10)
        self.time_slider.setValue(1)
        self.sliders_verticalLayout.addWidget(self.time_slider)
        
        self.hline_fst = QtWidgets.QFrame(self.sliders_frame)
        self.hline_fst.setFrameShape(QtWidgets.QFrame.HLine)
        self.hline_fst.setFrameShadow(QtWidgets.QFrame.Plain)
        self.hline_fst.setObjectName("hline_fst")
        self.sliders_verticalLayout.addWidget(self.hline_fst)
        
        self.speed_slider = BetterHSlider(self.sliders_frame)
        self.speed_slider.setObjectName("speed_slider")
        self.speed_slider.setText("Wind speed (m/s)")
        self.speed_slider.setDecimals(1)
        self.speed_slider.setRange(0,30)
        self.speed_slider.setValue(0)
        self.sliders_verticalLayout.addWidget(self.speed_slider)
        
        self.hline_snd = QtWidgets.QFrame(self.sliders_frame)
        self.hline_snd.setFrameShape(QtWidgets.QFrame.HLine)
        self.hline_snd.setFrameShadow(QtWidgets.QFrame.Plain)
        self.hline_snd.setObjectName("hline_snd")
        self.sliders_verticalLayout.addWidget(self.hline_snd)
        
        self.up_slider = BetterHSlider(self.sliders_frame)
        self.up_slider.setObjectName("up_slider")
        self.up_slider.setText("Updraft (m/s)")
        self.up_slider.setDecimals(1)
        self.up_slider.setRange(-10,10)
        self.up_slider.setValue(0)
        self.sliders_verticalLayout.addWidget(self.up_slider)
        
        self.hline_trd = QtWidgets.QFrame(self.sliders_frame)
        self.hline_trd.setFrameShape(QtWidgets.QFrame.HLine)
        self.hline_trd.setFrameShadow(QtWidgets.QFrame.Plain)
        self.hline_trd.setObjectName("hline_trd")
        self.sliders_verticalLayout.addWidget(self.hline_trd)
        
        self.gps_checkBox = QtWidgets.QCheckBox(self.sliders_frame)
        self.gps_checkBox.setObjectName("gps_checkBox")
        self.gps_checkBox.setText("GPS disabled")
        self.gps_checkBox.setChecked(False)
        self.sliders_verticalLayout.addWidget(self.gps_checkBox, 0, QtCore.Qt.AlignHCenter)
        self.verticalLayout_4.addLayout(self.sliders_verticalLayout)
        
        self.main_horizontalLayout.addWidget(self.sliders_frame)
        self.dial_frame = QtWidgets.QFrame(Dialog)
        self.dial_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.dial_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.dial_frame.setObjectName("dial_frame")
        
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.dial_frame)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        
        self.wind_dial = AngleSelector(self.dial_frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.wind_dial.sizePolicy().hasHeightForWidth())
        self.wind_dial.setSizePolicy(sizePolicy)
        self.wind_dial.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.wind_dial.setObjectName("wind_dial")
        self.wind_dial.setText("Wind direction (°)")
        self.horizontalLayout_2.addWidget(self.wind_dial)
        self.main_horizontalLayout.addWidget(self.dial_frame)
        self.horizontalLayout_3.addLayout(self.main_horizontalLayout)

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)
        
        

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Gaia", "Gaia"))
        self.gps_checkBox.setText(_translate("Dialog", "GPS disabled"))



class Gaia(QWidget,Ui_Dialog):
    def __init__(self, parent: QWidget | None = ..., flags: QtCore.Qt.WindowFlags | QtCore.Qt.WindowType = QtCore.Qt.WindowType.Widget) -> None:
        super().__init__(parent, flags)
        self.setupUi(self)
    

class GaiaWindow(QMainWindow):
    def __init__(self, parent: QWidget | None = None, flags: QtCore.Qt.WindowFlags | QtCore.Qt.WindowType = QtCore.Qt.WindowType.Window) -> None:
        super().__init__(parent, flags)
        
        self.gaiaWidget = Gaia(self)
        
        self.setObjectName("Gaia dialog")
        self.setWindowTitle("Gaia dialog")
        self.setCentralWidget(self.gaiaWidget)
        
        self.ivy = IvyMessagesInterface("gaia")
        self.timer = QtCore.QTimer(self)
        
        self.timer.timeout.connect(self.__send_status)
        
        self.gaiaWidget.gps_checkBox.stateChanged.connect(self.__send_status)
        
        self.gaiaWidget.time_slider.valueChanged.connect(self.__send_status)
        self.gaiaWidget.speed_slider.valueChanged.connect(self.__send_status)
        self.gaiaWidget.up_slider.valueChanged.connect(self.__send_status)
        self.gaiaWidget.wind_dial.valueChanged.connect(self.__send_status)
        
        self.timer.start(5000)
        
    def setUpdateInterval(self,msec:int):
        self.timer.setInterval(msec)
        
    def updateInterval(self) -> int:
        return self.timer.interval()
        
        
    def __send_status(self):
        rad_wind_dir = self.gaiaWidget.wind_dial.value()*math.pi/180
        wind_speed = self.gaiaWidget.speed_slider.value()
        wind_up = self.gaiaWidget.up_slider.value()
        timescale = self.gaiaWidget.time_slider.value()
        
        msg = PprzMessage("ground","WORLD_ENV")
        msg["wind_east"] = - wind_speed * math.sin(rad_wind_dir)
        msg["wind_north"] = - wind_speed * math.cos(rad_wind_dir)
        msg["wind_up"] = wind_up
        msg["ir_contrast"] = 266
        msg["time_scale"] = timescale
        msg["gps_availability"] = 0 if self.gaiaWidget.gps_checkBox.isChecked() else 1
        
        self.ivy.send(msg,self.ivy.agent_name)
        
        
    def closeEvent(self, e: QtGui.QCloseEvent) -> None:
        self.ivy.shutdown()
        e.accept()
        
if __name__ == "__main__":
    app = QApplication([])
    window = GaiaWindow(flags=QtCore.Qt.WindowType.Window)
    window.show()
    app.exec()

