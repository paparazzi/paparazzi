#!/usr/bin/env python
#
# Copyright (C) 2012-2014 The Paparazzi Team
#               2015 Freek van Tienen <freek.v.tienen@gmail.com>
#               2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
from parrot_utils import ParrotUtils
import re
import argparse
from time import sleep

class Ardrone2(ParrotUtils):
    uav_name = 'Ardrone 2'
    config_file = '/data/config.ini'
    version_file = '/firmware/version.txt'

    # Read from config.ini
    def read_from_config(self, name):
        # Only read file once
        if self.config_content == '':
            self.config_content = self.execute_command('cat ' + self.config_file)

        # Search for the name
        search = re.search(name + '[^=]+=[\r\n\t ]([^\r\n\t ]+)',self.config_content)
        if search is None:
            return 'Unknown'
        else:
            return search.group(1)

    # Write to config
    def write_to_config(self, name, value):
        if self.read_from_config(name) == 'Unknown':
            self.execute_command('echo "' + name + ' = ' + value + '\" >> ' + self.config_file)
        else:
            self.execute_command('sed -i "s/\(' + name + ' *= *\).*/\\1' + value + '/g" ' + self.config_file)

    def uav_status(self):
        print('Parrot version:\t\t' + str(self.check_version()))
        print('Host address:\t\t' + self.address +
                ' (' + self.read_from_config('static_ip_address_base') +
                self.read_from_config('static_ip_address_probe') + ' after boot)')
        print('Serial number:\t\t' + self.read_from_config('drone_serial'))
        print('Network id:\t\t' + self.read_from_config('ssid_single_player'))
        print('Motor software:\t\t' +
                self.read_from_config('motor1_soft') + '\t' + self.read_from_config('motor2_soft') + '\t' +
                self.read_from_config('motor3_soft') + '\t' + self.read_from_config('motor4_soft'))
        print('Motor hardware:\t\t' +
                self.read_from_config('motor1_hard') + '\t' + self.read_from_config('motor2_hard') + '\t' +
                self.read_from_config('motor3_hard') + '\t' + self.read_from_config('motor4_hard'))
        sleep(2.0) #Wait running process reporting back lag
        print('Currently running:\t' + self.check_running())
        autorun = {'': 'Native', '0': 'Native', '1': 'Paparazzi'}
        if self.check_autoboot():
            print('Autorun at start:\tInstalled booting ' + autorun[self.read_from_config('start_paparazzi')])
        else:
            print('Autorun at start:\tNot installed')

        # Check if the vision framework is installed and running
        vision_framework = ""
        if self.check_vision_installed():
            vision_framework += "Installed"
        if self.check_vision_running():
            vision_framework += " and running"
        print('Vision framework:\t' + vision_framework)


    # Check if vision framework is installed
    def check_vision_installed(self):
        du_opt = self.execute_command('du -d 2 /data/video/opt')
        return '/data/video/opt/arm/gst' in du_opt or '/data/video/opt/arm/lib' in du_opt or '/data/video/opt/arm/tidsp-binaries-23.i3.8' in du_opt
    
    # Check if the vision framework is running
    def check_vision_running(self):
        du_opt = self.execute_command('du -d 2 /opt')
        return '/opt/arm/gst' in du_opt and '/opt/arm/lib' in du_opt and '/opt/arm/tidsp-binaries-23.i3.8' in du_opt
    
    # Check if autoboot is installed
    def check_autoboot(self):
        check_update = self.execute_command('grep "START_PAPARAZZI" /bin/check_update.sh')
        wifi_setup = self.execute_command('grep "BASE_ADRESS" /bin/wifi_setup.sh')
        if "START_PAPARAZZI" in check_update and "BASE_ADRESS" in wifi_setup:
            return True
        else:
            return False
    
    # Check if custom wifi_setup script is installed
    def check_wifi_setup(self):
        check_wifi = self.execute_command('grep "static_ip_address_base" /bin/wifi_setup.sh')
        if "static_ip_address_base" in check_wifi:
            return True
        else:
            return False
    
    # Install the vision framework
    def ardrone2_install_vision(self):
        print('Uploading GST')
        self.upload_file("arm_light.tgz")
        print(self.execute_command("cd /data/video && tar -xzf arm_light.tgz"))
        print(self.execute_command("rm -rf /data/video/arm_light.tgz"))
        print('Now Starting Vision')
        self.ardrone2_start_vision()
    
    # Remove the vision framework
    def ardrone2_remove_vision(self):
        self.execute_command("rm -rf /opt/arm")
        self.execute_command("rm -rf /lib/dsp")
        self.execute_command("rm -rf /data/video/opt")
    
    # Start the vision framework
    def ardrone2_start_vision(self):
        # Mount the directories
        self.execute_command("mkdir -p /opt/arm")
        self.execute_command("mkdir -p /lib/dsp")
        self.execute_command("mount --bind /data/video/opt/arm /opt/arm")
        self.execute_command("mount --bind /data/video/opt/arm/lib/dsp /lib/dsp")
        # Start The DSP programs
        self.execute_command("kill -9 `pidof program.elf`")
        self.execute_command("kill -9 `pidof gst-launch-0.10`")
        self.execute_command("export PATH=/opt/arm/gst/bin:$PATH")
        self.execute_command("export DSP_PATH=/opt/arm/tidsp-binaries-23.i3.8/")
        self.execute_command("/bin/dspbridge/cexec.out -T /opt/arm/tidsp-binaries-23.i3.8/baseimage.dof -v")
        self.execute_command("/bin/dspbridge/dynreg.out -r /opt/arm/tidsp-binaries-23.i3.8/m4venc_sn.dll64P -v")
        # Show result
        self.execute_command("ls -altr /opt/arm/gst/bin")

    # Install autoboot script
    def ardrone2_install_autoboot(self):
        print('Uploading autoboot script')
        self.upload_file("ardrone2/check_update.sh")
        print(self.execute_command("mv /data/video/check_update.sh /bin/check_update.sh"))
        print(self.execute_command("chmod 777 /bin/check_update.sh"))
    
    # Install network script
    def ardrone2_install_network_script(self):
        print('Uploading Wifi script')
        self.upload_file("ardrone2/wifi_setup.sh")
        print(self.execute_command("mv /data/video/wifi_setup.sh /bin/wifi_setup.sh"))
        print(self.execute_command("chmod 777 /bin/wifi_setup.sh"))
    
    # Install olsr deamon
    def ardrone2_install_olsrd(self):
        print('Uploading olsr deamon')
        self.upload_file("ardrone2/olsrd")
        self.upload_file("ardrone2/olsrd.conf")
        print(self.execute_command("mv /data/video/olsrd /bin/olsrd"))
        print(self.execute_command("chmod 777 /bin/olsrd"))
        print(self.execute_command("mkdir -p /etc/olsrd"))
        print(self.execute_command("mv /data/video/olsrd.conf /etc/olsrd"))
        print(self.execute_command("rm -f /var/run && ln -s /tmp /var/run")) # olsrd needs /var/run folder, symlinked to /tmp
    
    # Set network SSID
    def ardrone2_set_ssid(self, name):
        self.write_to_config('ssid_single_player', name)
        print('The network ID (SSID) of the ARDrone 2 is changed to ' + name)
    
    # Set IP address
    def ardrone2_set_ip_address(self, address):
        splitted_ip = address.split(".")
        self.write_to_config('static_ip_address_base', splitted_ip[0] + '.' + splitted_ip[1] + '.' + splitted_ip[2] + '.')
        self.write_to_config('static_ip_address_probe', splitted_ip[3])
        print('The IP Address of the ARDrone 2 is changed to ' + address)
    
    # Set wifi mode (0: master, 1: ad-hoc, 2: managed, *: master)
    def ardrone2_set_wifi_mode(self, mode):
        modes = { 'master' : '0', 'ad-hoc' : '1', 'managed' : '2', 'ad-hoc-olsr' : '3' }
        try:
            val = modes[mode]
        except:
            print('Unexpected wifi mode, setting to master (default)')
            val = modes['master']
        self.write_to_config('wifi_mode', val)
        print('The Wifi mode of the ARDrone2 is changed to ' + mode + ' (' + val + ')')
    
    # Set network channel
    def ardrone2_set_wifi_channel(self, chan):
        self.write_to_config('wifi_channel', chan)
        print('The network channel of the ARDrone 2 is changed to ' + chan)

    def init_extra_parser(self):
        # Parse custom arguments
        self.subparsers.add_parser('installvision', help='Install the vision framework')
        self.subparsers.add_parser('startvision', help='Start the vision framework')
        ss = self.subparsers.add_parser('upload_gst_module',
                help='Upload, configure and move a gstreamer0.10 module libXXX.so')
        ss.add_argument('file', help='Filename of *.so module')

        ss = self.subparsers.add_parser('networkid', help='Set the network ID(SSID) of the ARDrone 2')
        ss.add_argument('name', help='the new network ID(SSID)')
        ss = self.subparsers.add_parser('ipaddress', help='Set the IP address of the ARDrone 2')
        ss.add_argument('address', help='the new IP address')
        ss = self.subparsers.add_parser('wifimode', help='Set the Wifi mode the ARDrone 2')
        ss.add_argument('mode', help='the new Wifi mode', choices=['master', 'ad-hoc', 'managed', 'ad-hoc-olsr'])
        ss = self.subparsers.add_parser('configure_network', help='Configure the network on the ARDrone 2')
        ss.add_argument('name', help='the new network ID(SSID)')
        ss.add_argument('address', help='the new IP address')
        ss.add_argument('mode', help='the new Wifi mode', choices=['master', 'ad-hoc', 'managed', 'ad-hoc-olsr'])
        ss.add_argument('--channel', help='the wifi channel (auto or 1 to 11)', default='auto')
        ss = self.subparsers.add_parser('install_autostart', help='Install custom autostart script and set what to start on boot for the ARDrone 2')
        ss.add_argument('type', choices=['native', 'paparazzi'],
                help='what to start on boot')
        ss = self.subparsers.add_parser('autostart', help='Set what to start on boot for the ARDrone 2')
        ss.add_argument('type', choices=['native', 'paparazzi'],
                help='what to start on boot')

    def parse_extra_args(self, args):

        # Change the network ID
        if args.command == 'networkid':
            self.ardrone2_set_ssid(args.name)
            if raw_input("Shall I restart the ARDrone 2? (y/N) ").lower() == 'y':
                self.reboot()

        # Change the IP address
        elif args.command == 'ipaddress':
            self.ardrone2_set_ip_address(args.address)
            if raw_input("Shall I restart the ARDrone 2? (y/N) ").lower() == 'y':
                self.reboot()

        # Change the wifi mode
        elif args.command == 'wifimode':
            self.ardrone2_set_wifi_mode(args.mode)
            if raw_input("Shall I restart the ARDrone 2? (y/N) ").lower() == 'y':
                self.reboot()

        # Install and configure network
        elif args.command == 'configure_network':
            self.config_content = self.execute_command('cat ' + self.config_file)
            print('=== Current network setup ===')
            print('Network id:\t' + self.read_from_config('ssid_single_player'))
            print('Host:\t\t' + args.host + ' (' + self.read_from_config('static_ip_address_base') +
                  self.read_from_config('static_ip_address_probe') + ' after boot)')
            print('Mode:\t\t' + self.read_from_config('wifi_mode'))
            print('Channel:\t' + self.read_from_config('wifi_channel'))
            print('=============================')
            if self.check_wifi_setup():
                print('Custom Wifi script already installed')
                if raw_input("Shall I reinstall the Wifi script (y/N) ").lower() == 'y':
                    self.ardrone2_install_network_script()
            else:
                if raw_input("Shall I install custom Wifi script (recommanded) (y/N) ").lower() == 'y':
                    self.ardrone2_install_network_script()
            if raw_input("Shall I install olsrd (ad-hoc wireless mesh routing deamon) (y/N) ").lower() == 'y':
                self.ardrone2_install_olsrd()
            self.ardrone2_set_ssid(args.name)
            self.ardrone2_set_ip_address(args.address)
            self.ardrone2_set_wifi_mode(args.mode)
            self.ardrone2_set_wifi_channel(args.channel)
            self.config_content = self.execute_command('cat ' + self.config_file)
            print('== New network setup after boot ==')
            print('Network id:\t' + self.read_from_config('ssid_single_player'))
            print('Host:\t\t' + self.read_from_config('static_ip_address_base') +
                  self.read_from_config('static_ip_address_probe'))
            print('Mode:\t\t' + self.read_from_config('wifi_mode'))
            print('Channel:\t' + self.read_from_config('wifi_channel'))
            print('==================================')

            if raw_input("Shall I restart the ARDrone 2? (y/N) ").lower() == 'y':
                self.reboot()

        # Install and configure autostart
        elif args.command == 'install_autostart':
            if self.check_autoboot():
                print('Custom autostart script already installed')
                if raw_input("Shall I reinstall the autostart script (y/N) ").lower() == 'y':
                    self.ardrone2_install_autoboot()
            else:
                self.ardrone2_install_autoboot()
            autorun = {'native': '0', 'paparazzi': '1'}
            self.write_to_config('start_paparazzi', autorun[args.type])
            print('The autostart on boot is changed to ' + args.type)

            if raw_input("Shall I restart the ARDrone 2? (y/N) ").lower() == 'y':
                self.reboot()

        # Change the autostart
        elif args.command == 'autostart':
            autorun = {'native': '0', 'paparazzi': '1'}
            self.write_to_config('start_paparazzi', autorun[args.type])
            print('The autostart on boot is changed to ' + args.type)

        # Install Vision framework
        elif args.command == 'installvision':
            if self.check_vision_installed():
                print('Vision framework already installed')
                if raw_input("Shall I reinstall the vision framework? (y/N) ").lower() == 'y':
                    self.ardrone2_remove_vision()
                    self.ardrone2_install_vision()

            self.ardrone2_install_vision()
            print('Vision framework installed')

        # Start Vision framework
        elif args.command == 'startvision':
            if self.check_vision_running():
                print('Vision framework already started')
            else:
                if not self.check_vision_installed():
                    print('No vision framework installed')
                    if raw_input("Shall I install the vision framework? (y/N) ").lower() == 'y':
                        self.ardrone2_install_vision()

                if self.check_vision_installed():
                    self.ardrone2_start_vision()
                    print('Vision framework started')

        elif args.command == 'upload_gst_module':
            print('Uploading ...' + args.file)
            self.upload_file(args.file)
            self.execute_command("chmod 777 /data/video/" + args.file)
            self.execute_command("mv /data/video/" + args.file + " /data/video/opt/arm/gst/lib/gstreamer-0.10")
            if self.check_vision_running():
                print('Info: Vision framework already started')
            else:
                if not self.check_vision_installed():
                    print('Warning: No vision framework installed')
                    if raw_input("Warning: Shall I install the vision framework? (y/N) ").lower() == 'y':
                        self.ardrone2_install_vision()

                if self.check_vision_installed():
                    self.ardrone2_start_vision()
                    print('#pragma message: Vision framework started')
            print('#pragma message: Vision Plugin Uploaded and DSP Started.')


if __name__ == "__main__":
    ardrone2 = Ardrone2()
    ardrone2.parse_args()
    exit(0)


