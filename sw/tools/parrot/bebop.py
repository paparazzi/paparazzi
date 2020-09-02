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
from time import sleep

class Bebop(ParrotUtils):
    uav_name = 'Bebop'
    address = '192.168.42.1'
    version_file = '/version.txt'
    upload_path = '/data/ftp/'
    scripts_path = 'internal_000/scripts/'
    config_file = upload_path + scripts_path + 'pprz.conf'
    check_version_before_run = True
    update_time_before_run = True

    # Read from config file
    def read_from_config(self, name):
        self.config_content = self.execute_command('cat ' + self.config_file)

        # Search for the name
        search = re.search(name + '=([^\r\n\t ]+)',self.config_content)
        if search is None:
            return 'Unknown'
        else:
            return search.group(1)

    # Write to config
    def write_to_config(self, name, value):
        if self.read_from_config(name) == 'Unknown':
            self.execute_command('echo "' + name + '=' + value + '\" >> ' + self.config_file)
        else:
            self.execute_command('sed -i "s/\(' + name + ' *= *\).*/\\1' + value + '/g" ' + self.config_file)

    def uav_status(self):
        print('Parrot version:\t\t' + str(self.check_version()))
        join = {'Unknown': 'No (Master)', '0': 'No (Master)', '1': 'Yes (Managed)'}
        print('Join Wifi:\t\t' + join[self.read_from_config('JOIN_WIFI')])
        print('Network id:\t\t' + self.read_from_config('WIFI_SSID'))
        print('Wifi Amode:\t\t' + self.read_from_config('WIFI_AMODE'))
        print('IP Address:\t\t' + self.read_from_config('WIFI_ADDRESS'))
        print('Currently running:\t' + self.check_running())
        autorun = {'Unknown': 'Native (autorun not installed)', '0': 'Native', '1': 'Paparazzi'}
        print('Autorun at start:\t'+autorun[self.read_from_config('START_PPRZ')])

    def bebop_install_scripts(self):
        print('Installing Paparazzi scripts')
        self.upload_file('bebop/pprz.conf', self.scripts_path, kill_prog=False)
        self.upload_file('bebop/config_network.script', self.scripts_path, kill_prog=False)
        self.upload_file('bebop/button_switch', self.scripts_path, kill_prog=False)
        self.upload_file('bebop/pprzstarter', self.scripts_path, kill_prog=False)
        self.execute_command("mount -o remount,rw /")
        if self.check_connect2hub():
            self.execute_command("sed -i 's|connect2hub|pprzstarter|' /etc/init.d/rcS")
            self.execute_command("rm /data/ftp/internal_000/scripts/connect2hub")
        else:
            self.execute_command("sed -i 's|^exit 0|/data/ftp/internal_000/scripts/pprzstarter \& exit 0|' /etc/init.d/rcS")
        self.execute_command("chmod a+x /etc/init.d/rcS")
        self.execute_command("chmod a+x /data/ftp/internal_000/scripts/pprzstarter")
        self.execute_command("chmod a+x /data/ftp/internal_000/scripts/button_switch")
        self.execute_command("chmod a+x /data/ftp/internal_000/scripts/config_network.script")
        self.execute_command("dos2unix /data/ftp/internal_000/scripts/pprzstarter")
        self.execute_command("dos2unix /data/ftp/internal_000/scripts/button_switch")
        self.execute_command("dos2unix /data/ftp/internal_000/scripts/pprz.conf")
        self.execute_command("cp /bin/onoffbutton/shortpress_3.sh /bin/onoffbutton/shortpress_3.sh.backup")
        self.execute_command("echo '#!/bin/sh' > /bin/onoffbutton/shortpress_3.sh")
        self.execute_command("echo '' >> /bin/onoffbutton/shortpress_3.sh")
        self.execute_command("echo '/data/ftp/internal_000/scripts/button_switch' >> /bin/onoffbutton/shortpress_3.sh")

    def bebop_uninstall_scripts(self):
        print('Uninstalling Paparazzi scripts')
        self.execute_command("mount -o remount,rw /")
        self.execute_command("sed -i 's|^/data/ftp/internal_000/scripts/pprzstarter \& exit 0|exit 0|' /etc/init.d/rcS")
        self.execute_command("chmod a+x /etc/init.d/rcS")
        self.execute_command("mv /bin/onoffbutton/shortpress_3.sh.backup /bin/onoffbutton/shortpress_3.sh")
        self.execute_command("rm -rf /data/ftp/internal_000/scripts/*")

    def check_autoboot(self):
        pprzstarter = self.execute_command('grep "pprzstarter" /etc/init.d/rcS')
        if "pprzstarter" in pprzstarter:
            return True
        else:
            return False

    def check_connect2hub(self):
        connect2hub = self.execute_command('grep "connect2hub" /etc/init.d/rcS')
        if "connect2hub" in connect2hub:
            return True
        else:
            return False

    def bebop_set_ssid(self, name):
        '''
        Set network SSID (of the router to join, not the Bebop SSID in master mode)
        '''
        self.write_to_config('WIFI_SSID', name)
        print('The network ID (SSID) to be joined is changed to ' + name)

    def bebop_set_wifi_mode(self, mode):
        '''
        Set Wifi mode, master or managed
        '''
        mode_id = { 'master': '0', 'managed': '1' }
        self.write_to_config('JOIN_WIFI', mode_id[mode])
        print('The Wifi mode is now ' + mode)

    def bebop_set_address(self, address):
        '''
        Set static IP or dhcp
        '''
        self.write_to_config('WIFI_ADDRESS', address)
        print('The IP address is now ' + address)

    def bebop_set_wifi_key(self, amode, key):
        '''
        Set encryption mode and key
        '''
        self.write_to_config('WIFI_AMODE', amode)
        self.write_to_config('WIFI_KEY', key)
        print('The encryption mode is ' + amode + ' with key ' + key)

    def bebop_shutdown(self):
        '''
        Proper bebop shutdown
        '''
        print("Shuting down Bebop (restart by hand if needed)")
        self.execute_command('/bin/ardrone3_shutdown.sh', timeout=1)

    def reboot(self):
        '''
        Custom reboot, in fact a proper shutdown as simple reboot seems too brutal
        Restart has to be done by hand
        '''
        self.bebop_shutdown()

    def init_extra_parser(self):

        # Parse the extra arguments
        self.parser.add_argument('--min_version', metavar='MIN', default='3.3.0',
                help='force minimum version allowed')
        self.parser.add_argument('--max_version', metavar='MAX', default='4.4.2',
                help='force maximum version allowed')

        ss = self.subparsers.add_parser('networkid', help='Set the network ID (SSID) to join in managed mode')
        ss.add_argument('name', help='The new network ID (SSID)')

        ss = self.subparsers.add_parser('wifimode', help='Set the Wifi mode the Bebop 1 or 2')
        ss.add_argument('mode', help='The new Wifi mode', choices=['master', 'managed'])

        ss = self.subparsers.add_parser('address', help='Set the IP address, static or dhcp')
        ss.add_argument('address', help="The new IP address (static) or 'dhcp'")

        ss = self.subparsers.add_parser('wifikey', help='Set the Wifi encryption')
        ss.add_argument('amode', help="Encryption mode ('none' to disable, or available modes are: open|shared|openshared|wpa|wpapsk|wpa2|wpa2psk|wpanone|ftpsk)")
        ss.add_argument('key', help="Encryption key (anything when 'amode' is set to 'none')")

        ss = self.subparsers.add_parser('configure_network', help='Configure the network on the Bebop 1 or 2')
        ss.add_argument('name', help='The network ID (SSID) to join in managed mode')
        ss.add_argument('mode', help='The new Wifi mode', choices=['master', 'managed'])
        ss.add_argument('address', help="The new IP address (static) or 'dhcp'")

        ss = self.subparsers.add_parser('install_autostart', help='Install custom autostart script and set what to start on boot for the Bebop 1 or 2')
        ss.add_argument('type', choices=['native', 'paparazzi'],
                help='what to start on boot')

        ss = self.subparsers.add_parser('autostart', help='Set what to start on boot for the Bebop 1 or 2')
        ss.add_argument('type', choices=['native', 'paparazzi'],
                help='what to start on boot')

        ss = self.subparsers.add_parser('uninstall_autostart', help='Remove custom autostart scripts')

    def parse_extra_args(self, args):

        # Change the network ID
        if args.command == 'networkid':
            self.bebop_set_ssid(args.name)
            if raw_input("Shall I restart the Bebop? (y/N) ").lower() == 'y':
                self.reboot()

        # Change the wifi mode
        elif args.command == 'wifimode':
            self.bebop_set_wifi_mode(args.mode)
            if raw_input("Shall I restart the Bebop? (y/N) ").lower() == 'y':
                self.reboot()

        # Change the wifi mode
        elif args.command == 'address':
            if not (self.is_ip(args.address) or args.address == 'dhcp'):
                print("Invalid address or dhcp option. Leaving.")
                return
            self.bebop_set_address(args.address)
            if raw_input("Shall I restart the Bebop? (y/N) ").lower() == 'y':
                self.reboot()

        # Change the wifi encryption mode
        elif args.command == 'wifikey':
            self.bebop_set_wifi_key(args.amode, args.key)
            if raw_input("Shall I restart the Bebop? (y/N) ").lower() == 'y':
                self.reboot()

        # Install and configure network
        elif args.command == 'configure_network':
            print('=== Current network setup ===')
            self.uav_status()
            print('=============================')
            if not (self.is_ip(args.address) or args.address == 'dhcp'):
                print("Invalid address or dhcp option. Leaving.")
                return
            if self.check_autoboot():
                print('Custom autostart (and network) script already installed')
                if raw_input("Shall I reinstall the autostart (and network) script (y/N) ").lower() == 'y':
                    self.bebop_install_scripts()
            else:
                print('Custom autostart (and network) script is required but is not installed')
                if raw_input("Shall I reinstall the autostart (and network) script (y/N) ").lower() == 'y':
                    self.bebop_install_scripts()
                else:
                    print('Scripts not installed, Leaving.')
                    return
            sleep(0.5)
            self.bebop_set_ssid(args.name)
            self.bebop_set_wifi_mode(args.mode)
            self.bebop_set_address(args.address)
            sleep(0.5)
            print('== New network setup after boot ==')
            self.uav_status()
            print('==================================')

            if raw_input("Shall I restart the Bebop? (y/N) ").lower() == 'y':
                self.reboot()

        # Install and configure autostart
        elif args.command == 'install_autostart':
            if self.check_autoboot():
                print('Custom autostart script already installed')
                if raw_input("Shall I reinstall the autostart script (y/N) ").lower() == 'y':
                    self.bebop_install_scripts()
            else:
                self.bebop_install_scripts()
            autorun = {'native': '0', 'paparazzi': '1'}
            self.write_to_config('START_PPRZ', autorun[args.type])
            print('The autostart on boot is changed to ' + args.type)

            if raw_input("Shall I restart the Bebop? (y/N) ").lower() == 'y':
                self.reboot()

        # Change the autostart
        elif args.command == 'autostart':
            autorun = {'native': '0', 'paparazzi': '1'}
            self.write_to_config('START_PPRZ', autorun[args.type])
            print('The autostart on boot is changed to ' + args.type)

        # Uninstall autostart
        elif args.command == 'uninstall_autostart':
            if self.check_autoboot():
                self.bebop_uninstall_scripts()
            else:
                print("Autostart script not found")

if __name__ == "__main__":
    bebop = Bebop()
    bebop.parse_args()
    exit(0)

