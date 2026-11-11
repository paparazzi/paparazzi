#!/usr/bin/env python3

import sys
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import distro
import os
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


def mark(symbol, color):
    # a small icon with a colored symbol, for the buttons
    pixmap = QPixmap(16, 16)
    pixmap.fill(Qt.transparent)
    painter = QPainter(pixmap)
    painter.setPen(QColor(color))
    painter.drawText(pixmap.rect(), Qt.AlignCenter, symbol)
    painter.end()
    return QIcon(pixmap)


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
            self.buttonlist.setEnabled(not self._everything)
            return True
        else:
            print(f'  {RED}{BOLD}✘ FAILED:{NC} {cmd}')
            print(f'{RED}    Check documentation: {docs}{NC}')
            print('')
            self._ok = False
            self.buttonlist.setEnabled(not self._everything)
            return False


    def view(self,link):
        webbrowser.open(link)

    def step(self, title):
        self._ok = True
        self._title = title
        print(f'\n{CYAN}{BOLD}{"═" * 80}')
        print(f'  {title}')
        print(f'{"═" * 80}{NC}\n')

    def done(self, success_msg, error_msg):
        if self._ok:
            print(f'\n{GREEN}{BOLD}  ✔ {success_msg}{NC}\n')
        else:
            print(f'\n{RED}{BOLD}  ✘ {error_msg}{NC}')
            print(f'{RED}    Check documentation: {docs}{NC}\n')

    def add_step(self, button, step):
        self.step_button[step] = button
        button.clicked.connect(lambda: self.run(step))

    def run(self, step):
        # run an install step, and mark its button: green ✔ succeeded, red ✘ failed
        step()
        self.step_button[step].setIcon(mark('✔', 'green') if self._ok else mark('✘', 'red'))

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
        elif distro_version == 24.04:
            self.execute('sudo -E apt-get install -y liblablgtk2-ocaml-dev')
            # the MAVLink generator uses future 0.18, which imports imp: removed from Python 3.12
            self.execute('sudo -E apt-get install -y python3-zombie-imp')
        if distro_version == 18.04:
            self.execute('wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | sudo apt-key add -')
            self.execute('sudo apt-add-repository -y \'deb https://apt.kitware.com/ubuntu/ bionic main\'')
        self.execute('sudo -E apt-get install -y cmake')
        self.execute('sudo -E apt-get install -y libboost-program-options-dev libboost-filesystem-dev')
        # SDL 1.2 for the joystick (sw/ground_segment/joystick, NPS joystick): not pulled in by paparazzi-dev on 26.04
        self.execute('sudo -E apt-get install -y libsdl1.2-dev')
        # the Bebop upload tools use telnetlib, removed from Python 3.13 (26.04): a copy of it from apt
        if distro_version >= 26.04:
            self.execute('sudo -E apt-get install -y python3-zombie-telnetlib')
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
        if distro_version == 22.04:
            # PprzGCS (Qt6) needs the xcb platform plugin, which Ubuntu 22.04 doesn't pull in
            self.execute('sudo -E apt-get -f -y install qt6-qpa-plugins')
        self.execute('sudo -E apt-get -f -y install python3-pyqt5.qtwebengine') # the documentation viewer of the Paparazzi Center
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
            # Try gazebo11 first, fallback to gazebo if it fails
            if not self.execute('sudo -E apt-get -f -y install gazebo11 libgazebo11-dev'):
                print(f'{YELLOW}  ℹ gazebo11 not available, trying fallback gazebo package...{NC}')
                self._ok = True  # Reset and try fallback
                self.execute('sudo -E apt-get -f -y install gazebo libgazebo-dev')
        self.execute('git submodule init && git submodule sync && git submodule update ./sw/ext/tudelft_gazebo_models')
        self.done('Gazebo Classic installed successfully', 'Gazebo Classic installation failed')

    def cmd_gazebo(self):
        self.step('Step 7b — Installing Gazebo (Harmonic, Jetty on 26.04)')
        if distro_version >= 22.04:
            self.execute('sudo -E apt-get update')
            self.execute('sudo -E apt-get -f -y install curl lsb-release gnupg')
            self.execute('sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg')
            self.execute('echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null')
            self.execute('sudo apt update')
            if distro_version >= 26.04:
                self.execute('sudo -E apt-get -f -y install gz-jetty')
                # gz-sim.pc of gz-sim10 requires exactly Qt6QuickControls2 6.4 (Ubuntu 26.04 has 6.10):
                # pkg-config then fails and the NPS build doesn't find the gz headers
                self.execute('sudo sed -i "s/Qt6QuickControls2 = 6.4/Qt6QuickControls2/" /usr/lib/x86_64-linux-gnu/pkgconfig/gz-sim.pc')
            else:
                self.execute('sudo -E apt-get -f -y install gz-harmonic')
        self.execute('git submodule init && git submodule sync && git submodule update ./sw/ext/tudelft_gazebo_models')
        self.done('Gazebo installed successfully', 'Gazebo installation failed')


    def cmd_bebopcv(self):
        self.step('Step 8 — Installing Bebop OpenCV')
        self.execute('git submodule init && git submodule sync && git submodule update ./sw/ext/opencv_bebop')
        if distro_version < 22.04:
            self.execute('sudo -E apt-get -f -y install libjpeg-turbo8-dev libpng-dev libtiff-dev zlib1g-dev libdc1394-22-dev')
        else:
            self.execute('sudo -E apt-get -f -y install libjpeg-turbo8-dev libpng-dev libtiff-dev zlib1g-dev libdc1394-dev')
        self.done('Bebop OpenCV installed successfully', 'Bebop OpenCV installation failed')

    def cmd_vlc(self):
        self.step('Step 9 — Installing VLC, Joystick & NatNet tools')
        self.execute('sudo -E apt-get -f -y install ffmpeg vlc gstreamer1.0-tools gstreamer1.0-plugins-good jstest-gtk default-jre')
        self.execute('sudo -E apt-get -f -y install python3-opencv') # RTP-viewer
        if distro_version <= 20.04:
            # no apt package of pymap3d on 20.04, where pip may still install in the system Python
            self.execute('sudo -E apt-get install -y python3-pip')
            self.execute('python3 -m pip install pymap3d') # Moving-Base
        else:
            self.execute('sudo -E apt-get -f -y install python3-pymap3d') # Moving-Base
        if distro_version < 26.04:
            self.execute('sudo -E apt-get -f -y install python3-future') # MAVLink generator (no apt package on 26.04)
        self.done('VLC / Joystick / NatNet installed successfully', 'Some packages failed to install')


    def cmd_doc(self):
        self.view('https://paparazzi-uav.readthedocs.io')


    def cmd_go(self):
        self.step('Step 11 — Building and launching Paparazzi Center')
        # a fresh clone has nothing to clean, and make clean fails on its still empty pprzlink submodule
        if os.path.isfile('sw/ext/pprzlink/Makefile'):
            self.execute('make clean')
        self.execute('make -j1')
        self.done('Build succeeded', 'Build failed — not starting Paparazzi Center')
        if self._ok:
            self.execute('./paparazzi &')
            if self.close_after_make.isChecked():
                self.close()

    def cmd_everything(self):
        # every install step of this Ubuntu version, then build and start the Paparazzi Center
        steps = [self.cmd_repo, self.cmd_dev, self.cmd_arm, self.cmd_jsb]
        if distro_version >= 20.04:
            steps.append(self.cmd_gcs)
        steps.append(self.cmd_mcu)
        if distro_version <= 22.04:
            steps.append(self.cmd_gazebo_classic)
        if distro_version >= 22.04:
            steps.append(self.cmd_gazebo)
        steps += [self.cmd_bebopcv, self.cmd_vlc]
        failed = []
        self._everything = True
        self.buttonlist.setEnabled(False)
        try:
            for run_step in steps:
                self.run(run_step)
                if not self._ok:
                    failed.append(self._title)
            if failed:
                print(f'{RED}{BOLD}  ✘ Failed steps:{NC}')
                for title in failed:
                    print(f'{RED}    {title}{NC}')
                print(f'{RED}    Check documentation: {docs}{NC}\n')
            self.run(self.cmd_go)
        finally:
            self._everything = False
            self.buttonlist.setEnabled(True)

    def cmd_pdf(self):
        self.view('https://github.com/tudelft/coursePaparazzi/raw/master/paparazzi_crash_course.pdf')


    def __init__(self):
        QWidget.__init__(self)
        self._everything = False
        self.step_button = {}
        mainlayout = QVBoxLayout()

        os = QLabel(f"OS: {distro.name()} {distro.version()}")
        os.setFont(QFont("Times", 18, QFont.Bold))
        mainlayout.addWidget(os)

        photo = QLabel()
        photo.setPixmap(QPixmap("./data/pictures/Penguin.gif"))
        photo.setAlignment(Qt.AlignCenter)   # its own size, not stretched to the width of the window
        photo.setObjectName("photo")
        mainlayout.addWidget(photo)


        # Box with all buttons
        self.buttonlist = QGroupBox("Install")
        btn_layout = QVBoxLayout()
        self.buttonlist.setLayout(btn_layout)


        button0 = QPushButton('Everything')
        button0.setFont(QFont(button0.font().family(), -1, QFont.Bold))
        button0.clicked.connect(self.cmd_everything)
        btn_layout.addWidget(button0)

        button1 = QPushButton('1) Add repository')
        self.add_step(button1, self.cmd_repo)
        btn_layout.addWidget(button1)

        button2 = QPushButton('2) Paparazzi Dev')
        self.add_step(button2, self.cmd_dev)
        btn_layout.addWidget(button2)

        button3 = QPushButton('3) Compiler for autopilots')
        self.add_step(button3, self.cmd_arm)
        btn_layout.addWidget(button3)

        button4 = QPushButton('4) JSB Simulator')
        self.add_step(button4, self.cmd_jsb)
        btn_layout.addWidget(button4)

        button5 = QPushButton('5) Binary ground station')
        if distro_version < 20.04:
            button5.setDisabled(True)
        else:
            self.add_step(button5, self.cmd_gcs)
        btn_layout.addWidget(button5)

        button6 = QPushButton('6) Flash and debug MCU')
        self.add_step(button6, self.cmd_mcu)
        btn_layout.addWidget(button6)

        button7 = QPushButton('7a) Gazebo11 Classic')
        if distro_version > 22.04:
            button7.setDisabled(True)
        self.add_step(button7, self.cmd_gazebo_classic)
        btn_layout.addWidget(button7)

        button7b = QPushButton('7b) Gazebo (Harmonic/Jetty)')
        if distro_version >= 22.04:
            self.add_step(button7b, self.cmd_gazebo)
        else:
            button7b.setDisabled(True)
        btn_layout.addWidget(button7b)

        button8 = QPushButton('8) Bebop Opencv')
        self.add_step(button8, self.cmd_bebopcv)
        btn_layout.addWidget(button8)

        button9 = QPushButton('9) VLC + Joystick + Natnet')
        self.add_step(button9, self.cmd_vlc)
        btn_layout.addWidget(button9)

        button10 = QPushButton('10) View Documentation')
        button10.clicked.connect(self.cmd_doc)
        btn_layout.addWidget(button10)

        button11 = QPushButton('11) Make and start Pprz Center')
        self.add_step(button11, self.cmd_go)
        btn_layout.addWidget(button11)

        self.close_after_make = QCheckBox('Close the installer when\nthe Paparazzi Center starts')
        self.close_after_make.setChecked(True)
        btn_layout.addWidget(self.close_after_make)

        mainlayout.addWidget(self.buttonlist)

        self.setLayout(mainlayout)
        self.setWindowTitle("Install Paparazzi-UAV")

if __name__ == '__main__':
    app = QApplication([])
    w = InstallWindow()
    w.show()
    sys.exit(app.exec_())
