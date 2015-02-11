#!/usr/bin/env python
#
# Copyright (C) 2012-2014 The Paparazzi Team
#
# This file is part of Paparazzi.
#
# Paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# Paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.
#

from __future__ import print_function
import re
import argparse
import socket
import telnetlib
import os
import sys
from time import sleep
from ftplib import FTP
import ftplib

import parrot_utils

# Read from config.ini
def read_from_config(name, config=''):
    if config == '':
        config = parrot_utils.execute_command(tn, 'cat /data/config.ini')
    search = re.search(name + '[^=]+=[\r\n\t ]([^\r\n\t ]+)', config)
    if search is None:
        return ''
    else:
        return search.group(1)

# Write to config
def write_to_config(name, value):
    if read_from_config(name) == '':
        parrot_utils.execute_command(tn, 'echo "' + name + ' = ' + value + '\" >> /data/config.ini')
    else:
        parrot_utils.execute_command(tn, 'sed -i "s/\(' + name + ' *= *\).*/\\1' + value + '/g" /data/config.ini')

# Check if vision framework is installed
def check_vision_installed():
    du_opt = parrot_utils.execute_command(tn,'du -d 2 /data/video/opt')
    return '/data/video/opt/arm/gst' in du_opt or '/data/video/opt/arm/lib' in du_opt or '/data/video/opt/arm/tidsp-binaries-23.i3.8' in du_opt

# Check if the vision framework is running
def check_vision_running():
    du_opt = parrot_utils.execute_command(tn,'du -d 2 /opt')
    return '/opt/arm/gst' in du_opt and '/opt/arm/lib' in du_opt and '/opt/arm/tidsp-binaries-23.i3.8' in du_opt

# Check if autoboot is installed
def check_autoboot():
    check_update = parrot_utils.execute_command(tn,'grep "START_PAPARAZZI" /bin/check_update.sh')
    wifi_setup = parrot_utils.execute_command(tn,'grep "BASE_ADRESS" /bin/wifi_setup.sh')
    if "START_PAPARAZZI" in check_update and "BASE_ADRESS" in wifi_setup:
        return True
    else:
        return False

# Check if custom wifi_setup script is installed
def check_wifi_setup():
    check_wifi = parrot_utils.execute_command(tn,'grep "static_ip_address_base" /bin/wifi_setup.sh')
    if "static_ip_address_base" in check_wifi:
        return True
    else:
        return False

# Install the vision framework
def ardrone2_install_vision():
    print('Uploading GST')
    parrot_utils.uploadfile(ftp, "arm_light.tgz", file("bin/arm_light.tgz", "rb"))
    print(parrot_utils.execute_command(tn,"cd /data/video && tar -xzf arm_light.tgz"))
    print(parrot_utils.execute_command(tn,"rm -rf /data/video/arm_light.tgz"))
    print('Now Starting Vision')
    ardrone2_start_vision()

# Remove the vision framework
def ardrone2_remove_vision():
    parrot_utils.execute_command(tn,"rm -rf /opt/arm")
    parrot_utils.execute_command(tn,"rm -rf /lib/dsp")
    parrot_utils.execute_command(tn,"rm -rf /data/video/opt")

# Start the vision framework
def ardrone2_start_vision():
    # Mount the directories
    parrot_utils.execute_command(tn,"mkdir -p /opt/arm")
    parrot_utils.execute_command(tn,"mkdir -p /lib/dsp")
    parrot_utils.execute_command(tn,"mount --bind /data/video/opt/arm /opt/arm")
    parrot_utils.execute_command(tn,"mount --bind /data/video/opt/arm/lib/dsp /lib/dsp")
    # Start The DSP programs
    parrot_utils.execute_command(tn,"kill -9 `pidof program.elf`")
    parrot_utils.execute_command(tn,"kill -9 `pidof gst-launch-0.10`")
    parrot_utils.execute_command(tn,"export PATH=/opt/arm/gst/bin:$PATH")
    parrot_utils.execute_command(tn,"export DSP_PATH=/opt/arm/tidsp-binaries-23.i3.8/")
    parrot_utils.execute_command(tn,"/bin/dspbridge/cexec.out -T /opt/arm/tidsp-binaries-23.i3.8/baseimage.dof -v")
    parrot_utils.execute_command(tn,"/bin/dspbridge/dynreg.out -r /opt/arm/tidsp-binaries-23.i3.8/m4venc_sn.dll64P -v")
    # Show result
    parrot_utils.execute_command(tn,"ls -altr /opt/arm/gst/bin")

# Install autoboot script
def ardrone2_install_autoboot():
    print('Uploading autoboot script')
    parrot_utils.uploadfile(ftp, "check_update.sh", file("ardrone2/check_update.sh", "rb"))
    print(parrot_utils.execute_command(tn,"mv /data/video/check_update.sh /bin/check_update.sh"))
    print(parrot_utils.execute_command(tn,"chmod 777 /bin/check_update.sh"))

# Install network script
def ardrone2_install_network_script():
    print('Uploading Wifi script')
    parrot_utils.uploadfile(ftp, "wifi_setup.sh", file("ardrone2/wifi_setup.sh", "rb"))
    print(parrot_utils.execute_command(tn,"mv /data/video/wifi_setup.sh /bin/wifi_setup.sh"))
    print(parrot_utils.execute_command(tn,"chmod 777 /bin/wifi_setup.sh"))

# Install olsr deamon
def ardrone2_install_olsrd():
    print('Uploading olsr deamon')
    parrot_utils.uploadfile(ftp, "olsrd", file("ardrone2/olsrd", "rb"))
    parrot_utils.uploadfile(ftp, "olsrd.conf", file("ardrone2/olsrd.conf", "rb"))
    print(parrot_utils.execute_command(tn,"mv /data/video/olsrd /bin/olsrd"))
    print(parrot_utils.execute_command(tn,"chmod 777 /bin/olsrd"))
    print(parrot_utils.execute_command(tn,"mkdir -p /etc/olsrd"))
    print(parrot_utils.execute_command(tn,"mv /data/video/olsrd.conf /etc/olsrd"))
    print(parrot_utils.execute_command(tn,"rm -f /var/run && ln -s /tmp /var/run")) # olsrd needs /var/run folder, symlinked to /tmp

# Set network SSID
def ardrone2_set_ssid(name):
    write_to_config('ssid_single_player', name)
    print('The network ID (SSID) of the ARDrone 2 is changed to ' + name)

# Set IP address
def ardrone2_set_ip_address(address):
    splitted_ip = address.split(".")
    write_to_config('static_ip_address_base', splitted_ip[0] + '.' + splitted_ip[1] + '.' + splitted_ip[2] + '.')
    write_to_config('static_ip_address_probe', splitted_ip[3])
    print('The IP Address of the ARDrone 2 is changed to ' + address)

# Set wifi mode (0: master, 1: ad-hoc, 2: managed, *: master)
def ardrone2_set_wifi_mode(mode):
    modes = { 'master' : '0', 'ad-hoc' : '1', 'managed' : '2', 'ad-hoc-olsr' : '3' }
    try:
        val = modes[mode]
    except:
        print('Unexpected wifi mode, setting to master (default)')
        val = modes['master']
    write_to_config('wifi_mode', val)
    print('The Wifi mode of the ARDrone2 is changed to ' + mode + ' (' + val + ')')

# Set network channel
def ardrone2_set_wifi_channel(chan):
    write_to_config('wifi_channel', chan)
    print('The network channel of the ARDrone 2 is changed to ' + chan)

def ardrone2_status():
    config_ini = parrot_utils.execute_command(tn,'cat /data/config.ini')

    print('======================== ARDrone 2 Status ========================')
    print('Version:\t\t' + parrot_utils.check_version(tn, '/firmware'))
    print('Host:\t\t\t' + args.host + ' (' + read_from_config('static_ip_address_base', config_ini) +
          read_from_config('static_ip_address_probe', config_ini) + ' after boot)')
    print('Currently running:\t' + parrot_utils.check_running(tn))
    print('Serial number:\t\t' + read_from_config('drone_serial', config_ini))
    print('Network id:\t\t' + read_from_config('ssid_single_player', config_ini))
    print('Motor software:\t\t' +
          read_from_config('motor1_soft', config_ini) + '\t' + read_from_config('motor2_soft', config_ini) + '\t' +
          read_from_config('motor3_soft', config_ini) + '\t' + read_from_config('motor4_soft', config_ini))
    print('Motor hardware:\t\t' +
          read_from_config('motor1_hard', config_ini) + '\t' + read_from_config('motor2_hard', config_ini) + '\t' +
          read_from_config('motor3_hard', config_ini) + '\t' + read_from_config('motor4_hard', config_ini))

    autorun = {'': 'Native', '0': 'Native', '1': 'Paparazzi RAW', '2': 'Paparazzi SDK'}
    if check_autoboot():
        print('Autorun at start:\tInstalled booting ' + autorun[read_from_config('start_paparazzi', config_ini)])
    else:
        print('Autorun at start:\tNot installed')

    # Check if the vision framework is installed and running
    vision_framework = ""
    if check_vision_installed():
        vision_framework += "Installed"
    if check_vision_running():
        vision_framework += " and running"
    print('Vision framework:\t' + vision_framework)

    # Request the filesystem status
    print('\n======================== Filesystem Status ========================')
    print(parrot_utils.check_filesystem(tn))


# Parse the arguments
parser = argparse.ArgumentParser(description='ARDrone 2 python helper. Use ardrone2.py -h for help')
parser.add_argument('--host', metavar='HOST', default='192.168.1.1',
                    help='the ip address of ardrone2')
subparsers = parser.add_subparsers(title='Command to execute', metavar='command', dest='command')

# All the subcommands and arguments
subparsers.add_parser('status', help='Request the status of the ARDrone 2')
subparsers.add_parser('reboot', help='Reboot the ARDrone 2')
subparsers.add_parser('installvision', help='Install the vision framework')
subparser_upload_gst = subparsers.add_parser('upload_gst_module',
                                         help='Upload, configure and move a gstreamer0.10 module libXXX.so')
subparser_upload_gst.add_argument('file', help='Filename of *.so module')
subparser_upload_and_run = subparsers.add_parser('upload_file_and_run', help='Upload and run software (for instance the Paparazzi autopilot)')
subparser_upload_and_run.add_argument('file', help='Filename of an executable')
subparser_upload_and_run.add_argument('folder', help='Destination subfolder (raw or sdk for Paparazzi autopilot)')
subparser_upload = subparsers.add_parser('upload_file', help='Upload a file to the ARDrone 2')
subparser_upload.add_argument('file', help='Filename')
subparser_upload.add_argument('folder', help='Destination subfolder (base destination folder is /data/video)')
subparser_download = subparsers.add_parser('download_file', help='Download a file from the ARDrone 2')
subparser_download.add_argument('file', help='Filename (with the path on the local machine)')
subparser_download.add_argument('folder', help='Remote subfolder (base folder is /data/video)')
subparser_download_dir = subparsers.add_parser('download_dir', help='Download all files from a folder from the ARDrone 2')
subparser_download_dir.add_argument('dest', help='destination folder (on the local machine)')
subparser_download_dir.add_argument('folder', help='Remote subfolder (base folder is /data/video)')
subparser_rm_dir = subparsers.add_parser('rm_dir', help='Remove a directory and all its files from the ARDrone 2')
subparser_rm_dir.add_argument('folder', help='Remote subfolder (base folder is /data/video)')
subparser_insmod = subparsers.add_parser('insmod', help='Upload and insert kernel module')
subparser_insmod.add_argument('file', help='Filename of *.ko kernel module')
subparsers.add_parser('startvision', help='Start the vision framework')
subparser_start = subparsers.add_parser('start', help='Start a program on the ARDrone 2')
subparser_start.add_argument('program', help='the program to start')
subparser_kill = subparsers.add_parser('kill', help='Kill a program on the ARDrone 2')
subparser_kill.add_argument('program', help='the program to kill')
subparser_networkid = subparsers.add_parser('networkid', help='Set the network ID(SSID) of the ARDrone 2')
subparser_networkid.add_argument('name', help='the new network ID(SSID)')
subparser_ipaddress = subparsers.add_parser('ipaddress', help='Set the IP address of the ARDrone 2')
subparser_ipaddress.add_argument('address', help='the new IP address')
subparser_wifimode = subparsers.add_parser('wifimode', help='Set the Wifi mode the ARDrone 2')
subparser_wifimode.add_argument('mode', help='the new Wifi mode', choices=['master', 'ad-hoc', 'managed'])
subparser_configure_network = subparsers.add_parser('configure_network', help='Configure the network on the ARDrone 2')
subparser_configure_network.add_argument('name', help='the new network ID(SSID)')
subparser_configure_network.add_argument('address', help='the new IP address')
subparser_configure_network.add_argument('mode', help='the new Wifi mode', choices=['master', 'ad-hoc', 'managed', 'ad-hoc-olsr'])
subparser_configure_network.add_argument('--channel', help='the wifi channel (auto or 1 to 11)', default='auto')
subparser_install_autostart = subparsers.add_parser('install_autostart', help='Install custom autostart script and set what to start on boot for the ARDrone 2')
subparser_install_autostart.add_argument('type', choices=['native', 'paparazzi_raw', 'paparazzi_sdk'],
                                 help='what to start on boot')
subparser_autostart = subparsers.add_parser('autostart', help='Set what to start on boot for the ARDrone 2')
subparser_autostart.add_argument('type', choices=['native', 'paparazzi_raw', 'paparazzi_sdk'],
                                 help='what to start on boot')

args = parser.parse_args()

# Connect with telnet and ftp
tn, ftp = parrot_utils.connect(args.host)

# Check the ARDrone 2 status
if args.command == 'status':
    ardrone2_status()

# Reboot the drone
elif args.command == 'reboot':
    parrot_utils.reboot(tn)
    print('The ARDrone 2 is rebooting...')

# Kill a program
elif args.command == 'kill':
    parrot_utils.execute_command(tn,'killall -9 ' + args.program)
    print('Program "' + args.program + '" is now killed')

# Start a program
elif args.command == 'start':
    parrot_utils.execute_command(tn,args.start + ' &')
    print('Program "' + args.start + '" is now started')

# Change the network ID
elif args.command == 'networkid':
    ardrone2_set_ssid(args.name)

    if raw_input("Shall I restart the ARDrone 2? (y/N) ").lower() == 'y':
        parrot_utils.reboot(tn)

# Change the IP address
elif args.command == 'ipaddress':
    ardrone2_set_ip_address(args.address)

    if raw_input("Shall I restart the ARDrone 2? (y/N) ").lower() == 'y':
        parrot_utils.reboot(tn)

# Change the wifi mode
elif args.command == 'wifimode':
    ardrone2_set_wifi_mode(args.mode)
    
    if raw_input("Shall I restart the ARDrone 2? (y/N) ").lower() == 'y':
        parrot_utils.reboot(tn)

# Install and configure network
elif args.command == 'configure_network':
    config_ini = parrot_utils.execute_command(tn,'cat /data/config.ini')
    print('=== Current network setup ===')
    print('Network id:\t' + read_from_config('ssid_single_player', config_ini))
    print('Host:\t\t' + args.host + ' (' + read_from_config('static_ip_address_base', config_ini) +
          read_from_config('static_ip_address_probe', config_ini) + ' after boot)')
    print('Mode:\t\t' + read_from_config('wifi_mode', config_ini))
    print('Channel:\t' + read_from_config('wifi_channel', config_ini))
    print('=============================')
    if check_wifi_setup():
        print('Custom Wifi script already installed')
        if raw_input("Shall I reinstall the Wifi script (y/N) ").lower() == 'y':
            ardrone2_install_network_script()
    else:
        if raw_input("Shall I install custom Wifi script (recommanded) (y/N) ").lower() == 'y':
            ardrone2_install_network_script()
    if raw_input("Shall I install olsrd (ad-hoc wireless mesh routing deamon) (y/N) ").lower() == 'y':
        ardrone2_install_olsrd()
    ardrone2_set_ssid(args.name)
    ardrone2_set_ip_address(args.address)
    ardrone2_set_wifi_mode(args.mode)
    ardrone2_set_wifi_channel(args.channel)
    config_ini = parrot_utils.execute_command(tn,'cat /data/config.ini')
    print('== New network setup after boot ==')
    print('Network id:\t' + read_from_config('ssid_single_player', config_ini))
    print('Host:\t\t' + read_from_config('static_ip_address_base', config_ini) +
          read_from_config('static_ip_address_probe', config_ini))
    print('Mode:\t\t' + read_from_config('wifi_mode', config_ini))
    print('Channel:\t' + read_from_config('wifi_channel', config_ini))
    print('==================================')

    if raw_input("Shall I restart the ARDrone 2? (y/N) ").lower() == 'y':
        parrot_utils.reboot(tn)

# Install and configure autostart
elif args.command == 'install_autostart':
    if check_autoboot():
        print('Custom autostart script already installed')
        if raw_input("Shall I reinstall the autostart script (y/N) ").lower() == 'y':
            ardrone2_install_autoboot()
    else:
        ardrone2_install_autoboot()
    autorun = {'native': '0', 'paparazzi_raw': '1', 'paparazzi_sdk': '2'}
    write_to_config('start_paparazzi', autorun[args.type])
    print('The autostart on boot is changed to ' + args.type)

    if raw_input("Shall I restart the ARDrone 2? (y/N) ").lower() == 'y':
        parrot_utils.reboot(tn)

# Change the autostart
elif args.command == 'autostart':
    autorun = {'native': '0', 'paparazzi_raw': '1', 'paparazzi_sdk': '2'}
    write_to_config('start_paparazzi', autorun[args.type])
    print('The autostart on boot is changed to ' + args.type)

# Install Vision framework
elif args.command == 'installvision':
    if check_vision_installed():
        print('Vision framework already installed')
        if raw_input("Shall I reinstall the vision framework? (y/N) ").lower() == 'y':
            ardrone2_remove_vision()
            ardrone2_install_vision()

    ardrone2_install_vision()
    print('Vision framework installed')

# Start Vision framework
elif args.command == 'startvision':
    if check_vision_running():
        print('Vision framework already started')
    else:
        if not check_vision_installed():
            print('No vision framework installed')
            if raw_input("Shall I install the vision framework? (y/N) ").lower() == 'y':
                ardrone2_install_vision()

        if check_vision_installed():
            ardrone2_start_vision()
            print('Vision framework started')

elif args.command == 'upload_gst_module':
    print('Uploading ...' + args.file)
    parrot_utils.uploadfile(ftp, args.file, file(args.file, "rb"))
    parrot_utils.execute_command(tn,"chmod 777 /data/video/" + args.file)
    parrot_utils.execute_command(tn,"mv /data/video/" + args.file + " /data/video/opt/arm/gst/lib/gstreamer-0.10")
    if check_vision_running():
        print('Info: Vision framework already started')
    else:
        if not check_vision_installed():
            print('Warning: No vision framework installed')
            if raw_input("Warning: Shall I install the vision framework? (y/N) ").lower() == 'y':
                ardrone2_install_vision()

        if check_vision_installed():
            ardrone2_start_vision()
            print('#pragma message: Vision framework started')
    print('#pragma message: Vision Plugin Uploaded and DSP Started.')


elif args.command == 'insmod':
    modfile = parrot_utils.split_into_path_and_file(args.file)
    print('Uploading \'' + modfile[1])
    parrot_utils.uploadfile(ftp, modfile[1], file(args.file, "rb"))
    print(parrot_utils.execute_command(tn,"insmod /data/video/" + modfile[1]))

elif args.command == 'upload_file_and_run':
    # Split filename and path
    f = parrot_utils.split_into_path_and_file(args.file)

    print("Kill running " + f[1] + " and make folder " + args.folder)
    parrot_utils.execute_command(tn,"killall -9 " + f[1])
    sleep(1)
    parrot_utils.execute_command(tn, "mkdir -p /data/video/" + args.folder)
    print('Uploading \'' + f[1] + "\' from " + f[0] + " to " + args.folder)
    parrot_utils.uploadfile(ftp, args.folder + "/" + f[1], file(args.file, "rb"))
    sleep(0.5)
    parrot_utils.execute_command(tn, "chmod 777 /data/video/" + args.folder + "/" + f[1])
    parrot_utils.execute_command(tn, "/data/video/" + args.folder + "/" + f[1] + " > /dev/null 2>&1 &")
    print("#pragma message: Upload and Start of ap.elf to ARDrone2 Succes!")

elif args.command == 'upload_file':
    # Split filename and path
    f = parrot_utils.split_into_path_and_file(args.file)

    parrot_utils.execute_command(tn,"mkdir -p /data/video/" + args.folder)
    print('Uploading \'' + f[1] + "\' from " + f[0] + " to /data/video/" + args.folder)
    parrot_utils.uploadfile(ftp, args.folder + "/" + f[1], file(args.file, "rb"))
    print("#pragma message: Upload of " + f[1] + " to ARDrone2 Succes!")

elif args.command == 'download_file':
    # Split filename and path
    f = parrot_utils.split_into_path_and_file(args.file)
    # Open file and download
    try:
        file = open(args.file, 'wb')
        print('Downloading \'' + f[1] + "\' from " + args.folder + " to " + f[0])
        ftp.retrbinary("RETR " + args.folder + "/" + f[1], file.write)
        print("#pragma message: Download of " + f[1] + " from ARDrone2 Succes!")
    except IOError:
        print("#pragma message: Fail to open file " + args.file)
    except:
        os.remove(args.file)
        print("#pragma message: Download of " + f[1] + " from ARDrone2 Failed!")
    else:
        file.close()

elif args.command == 'download_dir':
    # Split filename and path
    files = parrot_utils.execute_command(tn, 'find /data/video/' + args.folder + ' -name \'*.*\'')
    # Create dest dir if needed
    if not os.path.exists(args.dest):
        os.mkdir(args.dest)
    # Open file and download
    for f in files.split():
        file_name = parrot_utils.split_into_path_and_file(f)
        file_source = args.folder + '/' + file_name[1]
        file_dest = args.dest + '/' + file_name[1]
        try:
            file = open(file_dest, 'wb')
            print('Downloading \'' + f + "\' to " + file_dest)
            ftp.retrbinary("RETR " + file_source, file.write)
        except IOError:
            print("#pragma message: Fail to open file " + file_dest)
        except:
            os.remove(file_dest)
            print("#pragma message: Download of " + f + " from ARDrone2 Failed!")
        else:
            file.close()
    print("#pragma message: End download of folder " + args.folder + " from ARDrone2")

elif args.command == 'rm_dir':
    # Split filename and path
    print("Deleting folder /data/video/" + args.folder + " from ARDrone2")
    print(parrot_utils.execute_command(tn, 'rm -r /data/video/' + args.folder))


# Close the telnet and python script
parrot_utils.disconnect(tn, ftp)
exit(0)

