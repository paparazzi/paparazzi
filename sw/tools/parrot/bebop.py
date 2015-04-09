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


# Read from config.ini TODO
def read_from_config(name, config=''):
    if config == '':
        config = execute_command('cat /data/config.ini')
    search = re.search(name + '[^=]+=[\r\n\t ]([^\r\n\t ]+)', config)
    if search is None:
        return ''
    else:
        return search.group(1)

# Write to config TODO
def write_to_config(name, value):
    if read_from_config(name) == '':
        execute_command('echo "' + name + ' = ' + value + '\" >> /data/config.ini')
    else:
        execute_command('sed -i "s/\(' + name + ' *= *\).*/\\1' + value + '/g" /data/config.ini')


def bebop_status():
    #config_ini = parrot_utils.execute_command(tn, 'cat /data/config.ini')

    print('======================== Bebop Status ========================')
    print('Version:\t\t' + parrot_utils.check_version(tn, ''))
    # Request the filesystem status
    print('\n=================== Filesystem Status =======================')
    print(parrot_utils.check_filesystem(tn))


# Parse the arguments
parser = argparse.ArgumentParser(description='Bebop python helper. Use bebop.py -h for help')
parser.add_argument('--host', metavar='HOST', default='192.168.42.1',
                    help='the ip address of bebop')
subparsers = parser.add_subparsers(title='Command to execute', metavar='command', dest='command')

# All the subcommands and arguments
subparsers.add_parser('status', help='Request the status of the Bebop')
subparsers.add_parser('reboot', help='Reboot the Bebop')
subparser_upload_and_run = subparsers.add_parser('upload_file_and_run', help='Upload and run software (for instance the Paparazzi autopilot)')
subparser_upload_and_run.add_argument('file', help='Filename of an executable')
subparser_upload_and_run.add_argument('folder', help='Destination subfolder (raw or sdk for Paparazzi autopilot)')
subparser_upload = subparsers.add_parser('upload_file', help='Upload a file to the Bebop')
subparser_upload.add_argument('file', help='Filename')
subparser_upload.add_argument('folder', help='Destination subfolder (base destination folder is /data/ftp)')
subparser_download = subparsers.add_parser('download_file', help='Download a file from the Bebop')
subparser_download.add_argument('file', help='Filename (with the path on the local machine)')
subparser_download.add_argument('folder', help='Remote subfolder (base folder is /data/ftp)')
subparser_download_dir = subparsers.add_parser('download_dir', help='Download all files from a folder from the Bebop')
subparser_download_dir.add_argument('dest', help='destination folder (on the local machine)')
subparser_download_dir.add_argument('folder', help='Remote subfolder (base folder is /data/ftp)')
subparser_rm_dir = subparsers.add_parser('rm_dir', help='Remove a directory and all its files from the Bebop')
subparser_rm_dir.add_argument('folder', help='Remote subfolder (base folder is /data/ftp)')
subparser_insmod = subparsers.add_parser('insmod', help='Upload and insert kernel module')
subparser_insmod.add_argument('file', help='Filename of *.ko kernel module')
subparser_start = subparsers.add_parser('start', help='Start a program on the Bebop')
subparser_start.add_argument('program', help='the program to start')
subparser_kill = subparsers.add_parser('kill', help='Kill a program on the Bebop')
subparser_kill.add_argument('program', help='the program to kill')

args = parser.parse_args()

# Connect with telnet and ftp
tn, ftp = parrot_utils.connect(args.host)

# Check the Bebop status
if args.command == 'status':
    print("Connected to Bebop at " + args.host)
    bebop_status()

# Reboot the drone
elif args.command == 'reboot':
    parrot_utils.reboot(tn)
    print('The Bebop is rebooting...')

# Kill a program
elif args.command == 'kill':
    parrot_utils.execute_command(tn, 'killall -9 ' + args.program)
    print('Program "' + args.program + '" is now killed')

# Start a program
elif args.command == 'start':
    parrot_utils.execute_command(tn, args.start + ' &')
    print('Program "' + args.start + '" is now started')


elif args.command == 'insmod':
    modfile = parrot_utils.split_into_path_and_file(args.file)
    print('Uploading \'' + modfile[1])
    parrot_utils.uploadfile(ftp, modfile[1], file(args.file, "rb"))
    print(parrot_utils.execute_command(tn, "insmod /data/ftp/" + modfile[1]))

elif args.command == 'upload_file_and_run':
    # Split filename and path
    f = parrot_utils.split_into_path_and_file(args.file)

    print("Kill running " + f[1] + " and make folder " + args.folder)
    parrot_utils.execute_command(tn,"killall -9 " + f[1])
    sleep(1)
    parrot_utils.execute_command(tn, "mkdir -p /data/ftp/" + args.folder)
    print('Uploading \'' + f[1] + "\' from " + f[0] + " to " + args.folder)
    parrot_utils.uploadfile(ftp, args.folder + "/" + f[1], file(args.file, "rb"))
    sleep(0.5)
    parrot_utils.execute_command(tn, "chmod 777 /data/ftp/" + args.folder + "/" + f[1])
    parrot_utils.execute_command(tn, "/data/ftp/" + args.folder + "/" + f[1] + " > /dev/null 2>&1 &")
    print("#pragma message: Upload and Start of ap.elf to Bebop Succes!")

elif args.command == 'upload_file':
    # Split filename and path
    f = parrot_utils.split_into_path_and_file(args.file)

    parrot_utils.execute_command(tn, "mkdir -p /data/ftp/" + args.folder)
    print('Uploading \'' + f[1] + "\' from " + f[0] + " to /data/ftp/" + args.folder)
    parrot_utils.uploadfile(ftp, args.folder + "/" + f[1], file(args.file, "rb"))
    print("#pragma message: Upload of " + f[1] + " to Bebop Succes!")

elif args.command == 'download_file':
    # Split filename and path
    f = parrot_utils.split_into_path_and_file(args.file)
    # Open file and download
    try:
        fd = open(args.file, 'wb')
        print('Downloading \'' + f[1] + "\' from " + args.folder + " to " + f[0])
        ftp.retrbinary("RETR " + args.folder + "/" + f[1], fd.write)
        print("#pragma message: Download of " + f[1] + " from Bebop Succes!")
    except IOError:
        print("#pragma message: Fail to open file " + args.file)
    except:
        os.remove(args.file)
        print("#pragma message: Download of " + f[1] + " from Bebop Failed!")
    else:
        fd.close()

elif args.command == 'download_dir':
    # Split filename and path
    files = parrot_utils.execute_command(tn, 'find /data/ftp/' + args.folder + ' -name \'*.*\'')
    # Create dest dir if needed
    if not os.path.exists(args.dest):
        os.mkdir(args.dest)
    # Open file and download
    for f in files.split():
        file_name = parrot_utils.split_into_path_and_file(f)
        file_source = args.folder + '/' + file_name[1]
        file_dest = args.dest + '/' + file_name[1]
        try:
            fd = open(file_dest, 'wb')
            print('Downloading \'' + f + "\' to " + file_dest)
            ftp.retrbinary("RETR " + file_source, fd.write)
        except IOError:
            print("#pragma message: Fail to open file " + file_dest)
        except:
            os.remove(file_dest)
            print("#pragma message: Download of " + f + " from Bebop Failed!")
        else:
            fd.close()
    print("#pragma message: End download of folder " + args.folder + " from Bebop")

elif args.command == 'rm_dir':
    # Split filename and path
    print("Deleting folder /data/ftp/" + args.folder + " from Bebop")
    print(parrot_utils.execute_command(tn, 'rm -r /data/ftp/' + args.folder))



# Close the telnet and python script
parrot_utils.disconnect(tn, ftp)
exit(0)
