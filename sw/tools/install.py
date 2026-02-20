#!/usr/bin/env python3

import sys
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import distro
import subprocess
import webbrowser

distro_version = float(distro.version())
docs = 'https://paparazzi-uav.readthedocs.io'

# Terminal colors
RED = '\033[0;31m'
GREEN = '\033[0;32m'
YELLOW = '\033[1;33m'
CYAN = '\033[0;36m'
BOLD = '\033[1m'
NC = '\033[0m'


class InstallWindow(QWidget):

    def execute(self,cmd):
        self.buttonlist.setEnabled(False)
        print(f'  {YELLOW}▸{NC} {cmd}')
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
        if p.returncode == 0:
            print(f'  {GREEN}✔{NC} {cmd} {GREEN}done{NC}')
            print('')
        else:
            print(f'  {RED}{BOLD}✘ FAILED:{NC} {cmd}')
            print(f'{RED}    Check documentation: {docs}{NC}')
            print('')
            self._ok = False
        self.buttonlist.setEnabled(True)
        

    def view(self,link):
        webbrowser.open(link)

    def step(self, title):
        self._ok = True
        print(f'\n{CYAN}{BOLD}{"═" * 80}')
        print(f'  {title}')
        print(f'{"═" * 80}{NC}\n')

    def done(self, success_msg, error_msg):
        if self._ok:
            print(f'\n{GREEN}{BOLD}  ✔ {success_msg}{NC}\n')
        else:
            print(f'\n{RED}{BOLD}  ✘ {error_msg}{NC}')
            print(f'{RED}    Check documentation: {docs}{NC}\n')

    def cmd_repo(self):
        self.step('Step 1 — Adding Paparazzi PPA repository')
        self.execute('sudo -E add-apt-repository -y ppa:paparazzi-uav/ppa && sudo -E apt-get update')
        self.done('Repository added successfully', 'Failed to add the repository')


    def cmd_dev(self):
        self.step('Step 2 — Installing Paparazzi development packages')
        self.execute('sudo -E apt-get -f -y install paparazzi-dev')
        self.execute('sudo -E apt-get -f -y install python-is-python3')
        # Missing
        if distro_version <= 20.04:
            self.execute('sudo -E apt-get install -y python3-lxml python3-numpy')
        elif distro_version >= 24.04:
            self.execute('sudo -E apt-get install -y liblablgtk2-ocaml-dev')
        if distro_version == 18.04:
            self.execute('wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | sudo apt-key add -')
            self.execute('sudo apt-add-repository \'deb https://apt.kitware.com/ubuntu/ bionic main\'')
        self.execute('sudo apt-get update && sudo apt-get install cmake')
        self.execute('sudo -E apt-get install -y libboost-program-options-dev libboost-filesystem-dev')
        if distro_version >= 24.04:
            self.execute('python3 -m pip install telnetlib3') # Required for bebop tools
        self.done('Paparazzi Dev installed successfully', 'Some development packages failed to install')


    def cmd_arm(self):
        self.step('Step 3 — Installing ARM compiler for autopilots')
        self.execute('sudo -E apt-get -f -y install paparazzi-dev')
        if distro_version >= 20.04:
            self.execute('sudo -E apt-get install -y python-is-python3 gcc-arm-none-eabi gdb-multiarch')
            # python3-serial?
        else:
            self.execute('sudo -E add-apt-repository -y ppa:team-gcc-arm-embedded/ppa')
            self.execute('sudo -E apt-get update')
            self.execute('sudo -E apt-get -f -y install gcc-arm-embedded')
        self.done('ARM compiler installed successfully', 'ARM compiler installation failed')

    def cmd_jsb(self):
        self.step('Step 4 — Installing JSBSim flight simulator')
        self.execute('sudo -E apt-get -f -y install paparazzi-jsbsim')
        self.done('JSBSim installed successfully', 'JSBSim installation failed')

    def cmd_gcs(self):
        self.step('Step 5 — Installing binary ground station')
        self.execute('sudo -E apt-get -f -y install pprzgcs')
        self.execute('sudo -E apt-get -f -y install python3-pyqt5.qtwebkit') # for the documentation
        self.done('Ground station installed successfully', 'Ground station installation failed')

    def cmd_mcu(self):
        self.step('Step 6 — Setting up MCU flash & debug tools')
        self.execute('sudo -E apt-get -f -y install dfu-util')
        self.execute('sudo -E cp conf/system/udev/rules/*.rules /etc/udev/rules.d/ && sudo -E udevadm control --reload-rules')
        self.done('MCU tools installed successfully', 'MCU tools installation failed')

    def cmd_gazebo_classic(self):
        self.step('Step 7a — Installing Gazebo 11 Classic')
        if distro_version <= 22.04:
            self.execute('sudo -E apt-get update')
            self.execute('sudo -E apt-get -f -y install lsb-release wget gnupg')
            self.execute('sudo sh -c \'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list\'')
            self.execute('wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -')
            self.execute('sudo apt update')
            self.execute('sudo -E apt-get -f -y install gazebo11 libgazebo11-dev')
        self.execute('git submodule init && git submodule sync && git submodule update ./sw/ext/tudelft_gazebo_models')
        self.done('Gazebo Classic installed successfully', 'Gazebo Classic installation failed')

    def cmd_gazebo(self):
        self.step('Step 7b — Installing Gazebo Harmonic')
        if distro_version >= 22.04:
            self.execute('sudo -E apt-get update')
            self.execute('sudo -E apt-get -f -y install curl lsb-release gnupg')
            self.execute('sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg')
            self.execute('echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null')
            self.execute('sudo apt update')
            self.execute('sudo -E apt-get -f -y install gz-harmonic')
        self.execute('git submodule init && git submodule sync && git submodule update ./sw/ext/tudelft_gazebo_models')
        self.done('Gazebo Harmonic installed successfully', 'Gazebo Harmonic installation failed')


    def cmd_bebopcv(self):
        self.step('Step 8 — Installing Bebop OpenCV')
        self.execute('git submodule init && git submodule sync && git submodule update ./sw/ext/opencv_bebop')
        if distro_version < 22.04:
            self.execute('sudo -E apt-get -f -y install cmake libjpeg-turbo8-dev libpng-dev libtiff-dev zlib1g-dev libdc1394-22-dev')
        else:
            self.execute('sudo -E apt-get -f -y install cmake libjpeg-turbo8-dev libpng-dev libtiff-dev zlib1g-dev libdc1394-dev')
        self.done('Bebop OpenCV installed successfully', 'Bebop OpenCV installation failed')

    def cmd_vlc(self):
        self.step('Step 9 — Installing VLC, Joystick & NatNet tools')
        self.execute('sudo -E apt-get -f -y install ffmpeg vlc jstest-gtk default-jre')
        self.execute('sudo -E apt-get install -y python3-pip')
        self.execute('python3 -m pip install future') # Required for MAVLink
        self.execute('python3 -m pip install pyquaternion ivy-python') # Required for NatNat
        self.execute('python3 -m pip install pymap3d') # Required for Moving-Base
        self.execute('python3 -m pip install opencv-python') # Required for RTP-viewer
        self.done('VLC / Joystick / NatNet installed successfully', 'Some packages failed to install')
        

    def cmd_doc(self):
        self.view('https://paparazzi-uav.readthedocs.io')


    def cmd_go(self):
        self.step('Step 11 — Building and launching Paparazzi Center')
        self.execute('make clean')
        self.execute('make -j1')
        self.done('Build succeeded', 'Build failed — not starting Paparazzi Center')
        if self._ok:
            self.execute('./paparazzi &')

    def cmd_pdf(self):
        self.view('https://github.com/tudelft/coursePaparazzi/raw/master/paparazzi_crash_course.pdf')


    def __init__(self):
        QWidget.__init__(self)
        mainlayout = QVBoxLayout()

        os = QLabel(f"OS: {distro.name()} {distro.version()}")
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
        if distro_version < 20.04:
            button5.setDisabled(True)
        else:
            button5.clicked.connect(self.cmd_gcs)
        btn_layout.addWidget(button5)

        button6 = QPushButton('6) Flash and debug MCU')
        button6.clicked.connect(self.cmd_mcu)
        btn_layout.addWidget(button6)

        button7 = QPushButton('7a) Gazebo11 Classic')
        if distro_version > 22.04:
            button7.setDisabled(True)
        button7.clicked.connect(self.cmd_gazebo_classic)
        btn_layout.addWidget(button7)

        button7b = QPushButton('7b) Gazebo Harmonic')
        if distro_version >= 22.04:
            button7b.clicked.connect(self.cmd_gazebo)
        else:
            button7b.setDisabled(True)
        btn_layout.addWidget(button7b)

        button8 = QPushButton('8) Bebop Opencv')
        button8.clicked.connect(self.cmd_bebopcv)
        btn_layout.addWidget(button8)

        button9 = QPushButton('9) VLC + Joystick + Natnet')
        button9.clicked.connect(self.cmd_vlc)
        btn_layout.addWidget(button9)

        button10 = QPushButton('10) View Documentation')
        button10.clicked.connect(self.cmd_doc)
        btn_layout.addWidget(button10)

        button11 = QPushButton('11) Make and start Pprz Center')
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
