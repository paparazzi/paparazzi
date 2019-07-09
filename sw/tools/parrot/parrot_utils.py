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
from __future__ import unicode_literals
from builtins import bytes
import socket
import telnetlib
import os
import sys
from ftplib import FTP
from time import sleep
import ftplib
import argparse
import re

class ParrotVersion(object):
    def __init__(self, s):
        try:
            self.raw = s
            ss = s.split(".")
            self.h = int(ss[0])
            self.m = int(ss[1])
            sss=ss[2].split("-RC")
            self.l = int(sss[0])
            if len(sss) > 1:
                self.rc = int(sss[1])
            else:
                self.rc = 0
        except:
            self.h = 0
            self.m = 0
            self.l = 0
            self.rc = 0
            self.raw = ''

    def version(self):
        return ( ( (self.h * 100 + self.m) * 100) + self.l) * 100 + self.rc

    def __str__(self):
        return str(self.h) + "." + str(self.m) + "." + str(self.l) + "." + str(self.rc)

    def __eq__(self, other):
        return self.version() == other.version()

    def __lt__(self, other):
        return self.version() < other.version()

    def __le__(self, other):
        return self.version() <= other.version()

    def __gt__(self, other):
        return self.version() > other.version()

    def __ge__(self, other):
        return self.version() >= other.version()

class ParrotUtils:

    # Programs that could be running on the drone
    runnable_programs = [
        ('program.elf', 'Parrot native UAV application'),
        ('dragon-prog', 'Parrot native UAV application'),
        ('ap.elf', 'Paparazzi'),
        ('gst-launch', 'GStreamer')
    ]

    # Default values
    version_file = '/update/version.txt'
    upload_path = '/data/video/'
    uav_name = 'Parrot UAV'
    address = '192.168.1.1'
    prompt = '# '
    check_version_before_run = False
    update_time_before_run = False

    # Initialize defaults
    def __init__(self):
        self.config_content = ''
        self.init_parser()
        self.init_extra_parser()

    # Connect with telnet and ftp, wait until login
    def connect(self):
        try:
            self.tn = telnetlib.Telnet(self.address, timeout=3)
            self.ftp = FTP(self.address)
            self.ftp.login()
            self.tn.read_until(bytes(self.prompt, 'utf-8'))
            return True
        except:
            print('Could not connect to the ' + self.uav_name + ' (address: ' + self.address + ')')
            print('Check if the ' + self.uav_name + ' is turned on and the computer is connected over wifi or bluetooth.')
            if self.address == '192.168.42.1':
                print("If you are using Bebop 1 or 2, don't forget pressing the power button 4 times after the Bebop has booted!\n")
                print("And if using Disco pressing the power button 2 times after aircraft powerup.")
                print("Or run the buttonpress script to get rid of this buttonpress annoyance.")
            exit(2)

    # Close the telnet and ftp
    def disconnect(self):
        self.tn.close()
        self.ftp.close()

    # Execute a command
    def execute_command(self, command, timeout=5):
        self.tn.write(bytes(command + '\n', 'utf-8'))
        s = self.tn.read_until(bytes(self.prompt, 'utf-8'), timeout)
        if s.endswith(b'[JS] $ '):
            s = s[len(command) + 2:-8]
        elif s.endswith(b'[RS.edu] $ '):
            s = s[len(command) + 2:-12]
        else:
            s = s[len(command) + 2:-4]
        return s

    # Upload ftp and catch memory-full error
    def upload(self, filename, content):
        try:
            self.ftp.storbinary("STOR " + filename, content)
        except ftplib.error_temp:
            print('FTP UPLOAD ERROR: Uploading the file to the ' + self.uav_name + ' failed!')
            print('Check if the Filesystem of the ' + self.uav_name + ' isn\'t full:')
            print(self.check_filesystem())
            sys.exit()
        except:
            print('FTP UPLOAD ERROR: Uploading the file to the ' + self.uav_name + ' failed!')
            print('FTP uploading failed with the following error: ', sys.exc_info()[0])
            print('Check if the Filesystem of the ' + self.uav_name + ' isn\'t full:')
            print(self.check_filesystem())
            sys.exit()

    # Download a file from the drone
    def download(self, filename, folder):
        # Split filename and path
        f = self.split_into_path_and_file(filename)
        # Open file and download
        try:
            fd = open(filename, 'wb')
            print('Downloading \'' + f[1] + "\' from " + folder + " to " + f[0])
            self.ftp.retrbinary("RETR " + folder + "/" + f[1], fd.write)
            print('#pragma message: Download of "' + f[1] + '" from the ' + self.uav_name + ' success!')
        except IOError:
            print('#error: Fail to open local file "' + filename + '"')
        except:
            os.remove(filename)
            print('#error: Download of "' + filename + '" from ' + self.uav_name + ' Failed!')
        else:
            fd.close()

    # Download a folder from the drone
    def download_folder(self, dest, folder):
        # find files
        files = self.execute_command('find ' + self.upload_path + folder + ' -name \'*.*\'')
        # Create dest dir if needed
        if not os.path.exists(dest):
            os.mkdir(dest)
        # Open file and download
        for f in files.split():
            file_dest = dest + '/' + file_name[1]
            self.download(file_dest, folder)
        print("#pragma message: End download of folder " + args.folder + " from " + self.uav_name)


    # helper function to split file name and path
    def split_into_path_and_file(self, name):
        if name.count('/') <= 0:
            return ["./", name]
        return name.rsplit('/', 1)

    def is_ip(address):
        try:
            socket.inet_aton(address)
            ip = True
        except socket.error:
            ip = False
        return ip

    # Check what currently is running on the drone
    def check_running(self):
        ps_aux = self.execute_command('ps')
        running = ""

        # Go trough all programings
        for prog in self.runnable_programs:
            if prog[0] in ps_aux:
                running += ' '+prog[1]+' ('+prog[0]+')'

        # Don't print the first space
        return running[1:]

    # Check the filesystem
    def check_filesystem(self):
        return self.execute_command('df -h')

    # Get the version of the drone
    def check_version(self):
        if self.version_file is not None:
            return ParrotVersion(self.execute_command('cat ' + self.version_file).strip())
        else:
            return "Unknown version"

    # Default status
    def status(self):
        print('======================== ' + self.uav_name + ' Status ========================')
        self.uav_status()

        print('\n======================== Filesystem Status ========================')
        print(self.check_filesystem())

    # Reboot the drone
    def reboot(self):
        self.execute_command('reboot', timeout=1)
        print('The ' + self.uav_name + ' is now rebooting')

    # Kill a running program
    def kill_program(self, name):
        self.execute_command('killall -9 ' + name)
        print('Program "' + name + '" is now killed')

    # Start a new program
    def start_program(self, name):
        self.execute_command('chmod 777 ' + name)
        self.execute_command(name + ' > /dev/null 2>&1 &')
        print('Program "' + name + '" is now started')

    # Create a new directory
    def create_directory(self, name):
        self.execute_command('mkdir -p ' + name)
        print('Created new directory "' + name + '"')

    # Remove a directory
    def remove_directory(self, name):
        self.execute_command('rm -r ' + name)
        print('Removed directory "' + name + '"')

    # Upload a new file
    def upload_file(self, name, folder="", kill_prog=True):
        f = self.split_into_path_and_file(name)

        if kill_prog:
            # First kill the running program
            self.kill_program(f[1])
            sleep(1)

        # Make the upload directory and upload the file
        self.create_directory(self.upload_path + folder)
        if len(folder) > 0:
            self.upload(folder + '/' + f[1], open(name, "rb"))
        else:
            self.upload(f[1], open(name, "rb"))
        sleep(0.5)
        print('Succesfully uploaded "' + name + '" to folder "' + folder + '"')

    # Upload and run a new program
    def upload_and_run(self, name, folder, min_ver=None, max_ver=None):
        if self.check_version_before_run and min_ver is not None and max_ver is not None:
            v = self.check_version()
            print("Checking " + self.uav_name + " firmware version... " + str(v))
            if ((not v == ParrotVersion('0.0.0.0')) and ((v < ParrotVersion(min_ver)) or (v > ParrotVersion(max_ver)))):
                print("Error: please upgrade your " + self.uav_name + " firmware to version between " + min_ver + " and " + max_ver + "!")
                return

        f = self.split_into_path_and_file(name)

        # Upload the file
        self.upload_file(name, folder)

        if self.update_time_before_run:
            from datetime import datetime
            self.execute_command("date --set '" + datetime.now().strftime('%Y-%m-%d %H:%M:%S') + "'")
            print("Set date on " + self.uav_name + " to " + datetime.now().strftime('%Y-%m-%d %H:%M:%S'))

        # Make the file executable and execute it
        self.start_program(self.upload_path + folder + '/' + f[1])
        print('#pragma message: Succesfully started "' + f[1] + '" on ' + self.uav_name)

    def insmod(self, modname):
        f = parrot_utils.split_into_path_and_file(modname)
        print('Uploading \'' + f[1])
        self.upload_file(modname)
        print(self.execute_command("insmod " + self.upload_path + '/' + modfile[1]))

#####################################################################

    # Main argument parser setup
    def init_parser(self):
        self.parser = argparse.ArgumentParser(description=self.uav_name + ' python helper. Use ' + sys.argv[0] + ' -h for help')
        self.parser.add_argument('--host', metavar='HOST', default=self.address,
                            help='the ip address of ' + self.uav_name)
        self.subparsers = self.parser.add_subparsers(title='Command to execute', metavar='command', dest='command')

        # Add commands
        self.subparsers.add_parser('status', help='Request the status of the ' + self.uav_name)
        self.subparsers.add_parser('reboot', help='Reboot the ' + self.uav_name)

        ss = self.subparsers.add_parser('kill', help='Kill a program on the ' + self.uav_name)
        ss.add_argument('program', help='The program to kill')

        ss = self.subparsers.add_parser('start', help='Start a program on the ' + self.uav_name)
        ss.add_argument('program', help='The program to start (base folder is the ftp folder)')

        ss = self.subparsers.add_parser('upload', help='Upload a file to the ' + self.uav_name)
        ss.add_argument('file', help='Filename')
        ss.add_argument('folder', help='Destination subfolder (base folder is the ftp folder)')

        ss = self.subparsers.add_parser('download', help='Download a file from the ' + self.uav_name)
        ss.add_argument('file', help='Remote filename (could include folder)')
        ss.add_argument('save_file', help='Destination file on local computer')

        ss = self.subparsers.add_parser('download_dir', help='Download all files from a folder from the ' + self.uav_name)
        ss.add_argument('dest', help='destination folder (on the local machine)')
        ss.add_argument('folder', help='Remote subfolder (base folder is the ftp folder)')

        ss = self.subparsers.add_parser('upload_file_and_run', help='Upload and run software (for instance the Paparazzi autopilot)')
        ss.add_argument('file', help='Filename of an executable')
        ss.add_argument('folder', help='Remote destination folder (base folder is the ftp folder)')

        ss = self.subparsers.add_parser('mkdir', help='Make a new directory on the ' + self.uav_name)
        ss.add_argument('folder', help='Remote subfolder (base folder is the ftp folder)')

        ss = self.subparsers.add_parser('rmdir', help='Remove a directory and all its files from the ' + self.uav_name)
        ss.add_argument('folder', help='Remote subfolder (base folder is the ftp folder)')

        ss = self.subparsers.add_parser('insmod', help='Upload and insert kernel module')
        ss.add_argument('file', help='Filename of *.ko kernel module')


    # Main function to parse arguments
    def parse_args(self):
        args = self.parser.parse_args()

        # First connect to the drone
        self.address = args.host
        if self.connect() == False:
            return False

        # Parse the command line arguments
        if args.command == 'status':
            self.status()
        elif args.command == 'reboot':
            self.reboot()
        elif args.command == 'kill':
            self.kill_program(args.program)
        elif args.command == 'start':
            self.start_program(self.upload_path + args.program)
        elif args.command == 'upload':
            self.upload_file(args.file, args.folder)
        elif args.command == 'download':
            self.download(args.file, args.save_file)
        elif args.command == 'upload_file_and_run':
            if hasattr(args, 'min_version') and hasattr(args, 'max_version'):
                self.upload_and_run(args.file, args.folder, args.min_version, args.max_version)
            else:
                self.upload_and_run(args.file, args.folder)
        elif args.command == 'mkdir':
            self.create_directory(self.upload_path + args.folder)
        elif args.command == 'rmdir':
            self.remove_directory(self.upload_path + args.folder)
        elif args.command == 'insmod':
            self.insmod(args.file)
        else:
            if self.parse_extra_args(args) == False:
                self.disconnect()
                return False

        # Disconnect
        self.disconnect()
        return True

