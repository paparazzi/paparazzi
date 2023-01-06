#!/usr/bin/env python3
import tkinter as tk
import lsb_release
import subprocess
import webbrowser

release = lsb_release.get_distro_information()
docs = 'https://paparazzi-uav.readthedocs.io'

window = tk.Tk()
window.title("Install Paparazzi-UAV")
window.minsize(300,600)

text = tk.Label(window, text="OS:" + release['ID'] + ' ' + release['RELEASE'])
text.pack()


# Logo
image = tk.PhotoImage(file="./data/pictures/Penguin.gif")
img = tk.Label(window, image=image)
img.pack()


def execute(cmd):
    print(cmd)
    p = subprocess.Popen(cmd, shell=True)
    p.communicate() # wait until ready
    print('=================================================')
    print(cmd)
    print('READY (verify above)')
    print('Check documentation to resolve problems:', docs)
    print('=================================================')

def view(link):
    webbrowser.open(link)

def cmd_repo():
    execute('sudo add-apt-repository -y ppa:paparazzi-uav/ppa && sudo apt-get update')

btn_repo = tk.Button(window, text="1) Add repository", command=cmd_repo)
btn_repo.pack()

def cmd_dev():
    execute('sudo apt-get -f -y install paparazzi-dev')
    # Missing
    if float(release['RELEASE']) <= 20.04:
        execute('sudo apt-get -f -y install python3-lxml python3-numpy')

btn_repo = tk.Button(window, text="2) Paparazzi Dev", command=cmd_dev)
btn_repo.pack()

def cmd_arm():
    execute('sudo apt-get -f -y install paparazzi-dev')
    if float(release['RELEASE']) >= 20.04:
        execute('sudo apt-get -f -y install python-is-python3 gcc-arm-none-eabi gdb-multiarch')
        # python3-serial?
    else:
        execute('sudo add-apt-repository -y ppa:team-gcc-arm-embedded/ppa')
        execute('sudo apt-get update')
        execute('sudo apt-get -f -y install gcc-arm-embedded')

btn_arm = tk.Button(window, text="3) Compiler for autopilots", command=cmd_arm)
btn_arm.pack()

def cmd_jsb():
    execute('sudo apt-get -f -y install paparazzi-jsbsim')

btn_repo = tk.Button(window, text="4) JSB Simulator", command=cmd_jsb)
btn_repo.pack()

def cmd_gcs():
    execute('sudo apt-get -f -y install pprzgcs')

if float(release['RELEASE']) < 20.04:
	btn_gcs = tk.Button(window, text="5) Binary ground station", state=tk.DISABLED)
else:
	btn_gcs = tk.Button(window, text="5) Binary ground station", command=cmd_gcs)
btn_gcs.pack()

def cmd_mcu():
    execute('sudo apt-get -f -y install dfu-util')
    execute('sudo cp conf/system/udev/rules/*.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules')

btn_mcu = tk.Button(window, text="6) Flash and debug MCU", command=cmd_mcu)
btn_mcu.pack()

def cmd_gazebo():
    if float(release['RELEASE']) <= 20.04:
        execute('sudo apt-get -f -y install gazebo9 libgazebo9-dev')
    else:
        execute('sudo apt-get -f -y install gazebo libgazebo-dev')
    execute('git submodule init && git submodule sync && git submodule update ./sw/ext/tudelft_gazebo_models')


if float(release['RELEASE']) <= 20.04:
	btn_gazebo = tk.Button(window, text="7) Gazebo9", command=cmd_gazebo)
else:
	btn_gazebo = tk.Button(window, text="7) Gazebo11", command=cmd_gazebo)
btn_gazebo.pack()

def cmd_bebopcv():
    execute('git submodule init && git submodule sync && git submodule update ./sw/ext/opencv_bebop')
    execute('sudo apt-get -f -y install cmake libjpeg-turbo8-dev libpng-dev libtiff-dev zlib1g-dev libdc1394-22-dev')

btn_bcv = tk.Button(window, text="8) Bebop Opencv", command=cmd_bebopcv)
btn_bcv.pack()

def cmd_vlc():
    execute('sudo apt-get -f -y install ffmpeg vlc jstest-gtk default-jre')
    execute('sudo apt-get -f -y install python3-pip')
    if float(release['RELEASE']) <= 20.04:
        execute('pip3 install pyquaternion ivy-python') # Required for NatNat
    else:
        execute('pip3 install pyquaternion ivy') # Required for NatNat

btn_vlc = tk.Button(window, text="9) VLC + Joystick + Natnet", command=cmd_vlc)
btn_vlc.pack()

def cmd_doc():
    view(docs)

btn_doc = tk.Button(window, text="10) View Documentation", command=cmd_doc)
btn_doc.pack()


def cmd_go():
    execute('make clean')
    execute('make -j1')
    execute('./start.py &')
    window.destroy()

btn_doc = tk.Button(window, text="11) Make and Run", command=cmd_go)
btn_doc.pack()



#    

# Start the event loop.
window.mainloop()

