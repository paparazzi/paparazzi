#!/usr/bin/env python3

import sys
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import lsb_release
import subprocess
import webbrowser

release = lsb_release.get_distro_information()
docs = 'https://paparazzi-uav.readthedocs.io'


class InstallWindow(QWidget):

    def execute(self,cmd):
        self.buttonlist.setEnabled(False)
        print(cmd)
        p = subprocess.Popen(cmd, shell=True)
        busy = True
        while busy:
            try:
                QApplication.processEvents()
                p.communicate(timeout=1) # wait until ready
                busy = False
                #print('finished!')
            except subprocess.TimeoutExpired:
                busy = True
                #print('Tick')
        print('================================================================================')
        print(cmd)
        print('READY (verify above)')
        print('Check documentation to resolve problems:', docs)
        print('================================================================================')
        self.buttonlist.setEnabled(True)
        

    def view(self,link):
        webbrowser.open(link)

    def cmd_repo(self):
        self.execute('sudo -E add-apt-repository -y ppa:paparazzi-uav/ppa && sudo -E apt-get update')


    def cmd_dev(self):
        self.execute('sudo -E apt-get -f -y install paparazzi-dev')
        # Missing
        if float(release['RELEASE']) <= 20.04:
            self.execute('sudo -E apt-get install -y python3-lxml python3-numpy')

    def cmd_arm(self):
        self.execute('sudo -E apt-get -f -y install paparazzi-dev')
        if float(release['RELEASE']) >= 20.04:
            self.execute('sudo -E apt-get install -y python-is-python3 gcc-arm-none-eabi gdb-multiarch')
            # python3-serial?
        else:
            self.execute('sudo -E add-apt-repository -y ppa:team-gcc-arm-embedded/ppa')
            self.execute('sudo -E apt-get update')
            self.execute('sudo -E apt-get -f -y install gcc-arm-embedded')

    def cmd_jsb(self):
        self.execute('sudo -E apt-get -f -y install paparazzi-jsbsim')

    def cmd_gcs(self):
        self.execute('sudo -E apt-get -f -y install pprzgcs')

    def cmd_mcu(self):
        self.execute('sudo -E apt-get -f -y install dfu-util')
        self.execute('sudo -E cp conf/system/udev/rules/*.rules /etc/udev/rules.d/ && sudo -E udevadm control --reload-rules')

    def cmd_gazebo(self):
        self.execute('sudo -E apt-get -f -y install gazebo9 libgazebo9-dev')
        self.execute('git submodule init && git submodule sync && git submodule update ./sw/ext/tudelft_gazebo_models')


    def cmd_bebopcv(self):
        self.execute('git submodule init && git submodule sync && git submodule update ./sw/ext/opencv_bebop')
        self.execute('sudo -E apt-get -f -y install cmake libjpeg-turbo8-dev libpng-dev libtiff-dev zlib1g-dev libdc1394-22-dev')

    def cmd_vlc(self):
        self.execute('sudo -E apt-get -f -y install ffmpeg vlc jstest-gtk default-jre')
        self.execute('sudo -E apt-get install -y python3-pip')
        self.execute('python3 -m pip install pyquaternion ivy-python') # Required for NatNat

    def cmd_doc(self):
        self.view('https://paparazzi-uav.readthedocs.io')


    def cmd_go(self):
        self.execute('make clean')
        self.execute('make -j1')
        self.execute('./start.py &')

    def cmd_pdf(self):
        self.view('https://github.com/tudelft/coursePaparazzi/raw/master/paparazzi_crash_course.pdf')


    def __init__(self):
        QWidget.__init__(self)
        mainlayout = QVBoxLayout()

        os = QLabel()
        os.setText("OS: " + release['ID'] + ' ' + release['RELEASE'])
        os.setFont(QFont("Times", 18, QFont.Bold))
        mainlayout.addWidget(os)

        photo = QLabel()
        photo.setGeometry(QRect(0, 0, 350, 300))
        photo.setText("")
        photo.setPixmap(QPixmap("./data/pictures/Penguin.gif"))
        photo.setScaledContents(True)
        photo.setObjectName("photo")
        mainlayout.addWidget(photo)


        # Box with all buttons
        self.buttonlist = QGroupBox("Install")
        btn_layout = QVBoxLayout()
        self.buttonlist.setLayout(btn_layout)


        button1 = QPushButton('1) Add repository')
        button1.clicked.connect(self.cmd_repo)
        btn_layout.addWidget(button1)

        button2 = QPushButton('2) Paparazzi Dev')
        button2.clicked.connect(self.cmd_dev)
        btn_layout.addWidget(button2)

        button3 = QPushButton('3) Compiler for autopilots')
        button3.clicked.connect(self.cmd_arm)
        btn_layout.addWidget(button3)

        button4 = QPushButton('4) JSB Simulator')
        button4.clicked.connect(self.cmd_jsb)
        btn_layout.addWidget(button4)

        button5 = QPushButton('5) Binary ground station')
        if float(release['RELEASE']) < 20.04:
            button5.setDisabled(True)
        else:
            button5.clicked.connect(self.cmd_gcs)
        btn_layout.addWidget(button5)

        button6 = QPushButton('6) Flash and debug MCU')
        button6.clicked.connect(self.cmd_mcu)
        btn_layout.addWidget(button6)

        if float(release['RELEASE']) <= 20.04:
            button7 = QPushButton('7) Gazebo9')
        else:
            button7 = QPushButton('7) Gazebo11')
        button7.clicked.connect(self.cmd_gazebo)
        btn_layout.addWidget(button7)

        button8 = QPushButton('8) Bebop Opencv')
        button8.clicked.connect(self.cmd_bebopcv)
        btn_layout.addWidget(button8)

        button9 = QPushButton('9) VLC + Joystick + Natnet')
        button9.clicked.connect(self.cmd_vlc)
        btn_layout.addWidget(button9)

        button10 = QPushButton('10) View Documentation')
        button10.clicked.connect(self.cmd_doc)
        btn_layout.addWidget(button10)

        button11 = QPushButton('11) Make and ./start.py')
        button11.clicked.connect(self.cmd_go)
        btn_layout.addWidget(button11)

        mainlayout.addWidget(self.buttonlist)

        self.setLayout(mainlayout)
        self.setWindowTitle("Install Paparazzi-UAV")

if __name__ == '__main__':
    app = QApplication([])
    w = InstallWindow()
    w.show()
    sys.exit(app.exec_())
