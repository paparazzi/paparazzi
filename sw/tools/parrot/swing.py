#!/usr/bin/env python
#
# Copyright (C) 2012-2017 The Paparazzi Team
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
import os
from time import sleep

import parrot_utils

PROMPT = '$'
FTP_ROOT = '/data/edu/'


# Read from config.ini TODO
def read_from_config(name, config=''):
    if config == '':
        config = parrot_utils.execute_command('cat /data/config.ini', prompt=PROMPT)
    search = re.search(name + '[^=]+=[\r\n\t ]([^\r\n\t ]+)', config)
    if search is None:
        return ''
    else:
        return search.group(1)

# Write to config TODO
def write_to_config(name, value):
    if read_from_config(name) == '':
        parrot_utils.execute_command('echo "' + name + ' = ' + value + '\" >> /data/config.ini', prompt=PROMPT)
    else:
        parrot_utils.execute_command('sed -i "s/\(' + name + ' *= *\).*/\\1' + value + '/g" /data/config.ini', prompt=PROMPT)


def swing_status():
    #config_ini = parrot_utils.execute_command(tn, 'cat /data/config.ini', prompt=PROMPT)

    #print('==================== Swing Status ====================')
    #print('Version:\t\t' + str(parrot_utils.check_version(tn, '', prompt=PROMPT)))
    # Request the filesystem status
    print('\n=================== Filesystem Status =======================')
    print(parrot_utils.check_filesystem(tn, prompt=PROMPT))


# Parse the arguments
parser = argparse.ArgumentParser(description='Swing python helper. Use swing.py -h for help')
parser.add_argument('--host', metavar='HOST', default='192.168.4.1',
                    help='the ip address of Swing')
subparsers = parser.add_subparsers(title='Command to execute', metavar='command', dest='command')

# All the subcommands and arguments
subparsers.add_parser('status', help='Request the status of the Swing')
subparsers.add_parser('reboot', help='Reboot the Swing')
subparser_upload_and_run = subparsers.add_parser('upload_file_and_run', help='Upload and run software (for instance the Paparazzi autopilot)')
subparser_upload_and_run.add_argument('file', help='Filename of an executable')
subparser_upload_and_run.add_argument('folder', help='Destination subfolder (raw or sdk for Paparazzi autopilot)')
subparser_upload = subparsers.add_parser('upload_file', help='Upload a file to the Swing')
subparser_upload.add_argument('file', help='Filename')
subparser_upload.add_argument('folder', help='Destination subfolder (base destination folder is '+FTP_ROOT+')')
subparser_download = subparsers.add_parser('download_file', help='Download a file from the Swing')
subparser_download.add_argument('file', help='Filename (with the path on the local machine)')
subparser_download.add_argument('folder', help='Remote subfolder (base folder is '+FTP_ROOT+')')
subparser_download_dir = subparsers.add_parser('download_dir', help='Download all files from a folder from the Swing')
subparser_download_dir.add_argument('dest', help='destination folder (on the local machine)')
subparser_download_dir.add_argument('folder', help='Remote subfolder (base folder is '+FTP_ROOT+')')
subparser_rm_dir = subparsers.add_parser('rm_dir', help='Remove a directory and all its files from the Swing')
subparser_rm_dir.add_argument('folder', help='Remote subfolder (base folder is '+FTP_ROOT+')')
subparser_insmod = subparsers.add_parser('insmod', help='Upload and insert kernel module')
subparser_insmod.add_argument('file', help='Filename of *.ko kernel module')
subparser_start = subparsers.add_parser('start', help='Start a program on the Swing')
subparser_start.add_argument('program', help='the program to start')
subparser_kill = subparsers.add_parser('kill', help='Kill a program on the Swing')
subparser_kill.add_argument('program', help='the program to kill')

args = parser.parse_args()

# Connect with telnet and ftp
tn, ftp = parrot_utils.connect(args.host, prompt=PROMPT)

# Check the Swing status
if args.command == 'status':
    print("Connected to Swing at " + args.host)
    swing_status()

# Reboot the drone
elif args.command == 'reboot':
    parrot_utils.reboot(tn, prompt=PROMPT)
    print('The Swing is rebooting...')

# Kill a program
elif args.command == 'kill':
    parrot_utils.execute_command(tn, 'killall -9 ' + args.program, prompt=PROMPT)
    print('Program "' + args.program + '" is now killed')

# Start a program
elif args.command == 'start':
    parrot_utils.execute_command(tn, args.start + ' &', prompt=PROMPT)
    print('Program "' + args.start + '" is now started')


elif args.command == 'insmod':
    modfile = parrot_utils.split_into_path_and_file(args.file)
    print('Uploading \'' + modfile[1])
    parrot_utils.uploadfile(ftp, modfile[1], file(args.file, "rb"))
    print(parrot_utils.execute_command(tn, "insmod "+ FTP_ROOT + modfile[1], prompt=PROMPT))

elif args.command == 'upload_file_and_run':
    # Split filename and path
    f = parrot_utils.split_into_path_and_file(args.file)

    print("Kill running " + f[1] + " and make folder " + args.folder)
    parrot_utils.execute_command(tn,"killall -9 " + f[1], prompt=PROMPT)
    sleep(1)
    parrot_utils.execute_command(tn, "mkdir -p " + FTP_ROOT + args.folder, prompt=PROMPT)
    print('Uploading \'' + f[1] + "\' from " + f[0] + " to " + args.folder)
    parrot_utils.uploadfile(ftp, args.folder + "/" + f[1], file(args.file, "rb"))
    sleep(0.5)
    parrot_utils.execute_command(tn, "chmod 777 " + FTP_ROOT + args.folder + "/" + f[1], prompt=PROMPT)
    parrot_utils.execute_command(tn, FTP_ROOT + args.folder + "/" + f[1] + " > /dev/null 2>&1 &", prompt=PROMPT)
    print("#pragma message: Upload and Start of ap.elf to Swing Succes!")

elif args.command == 'upload_file':
    # Split filename and path
    f = parrot_utils.split_into_path_and_file(args.file)

    parrot_utils.execute_command(tn, "mkdir -p " + FTP_ROOT + args.folder, prompt=PROMPT)
    print('Uploading \'' + f[1] + "\' from " + f[0] + " to " + FTP_ROOT + args.folder)
    parrot_utils.uploadfile(ftp, args.folder + "/" + f[1], file(args.file, "rb"))
    print("#pragma message: Upload of " + f[1] + " to Swing Succes!")

elif args.command == 'download_file':
    # Split filename and path
    f = parrot_utils.split_into_path_and_file(args.file)
    # Open file and download
    try:
        fd = open(args.file, 'wb')
        print('Downloading \'' + f[1] + "\' from " + args.folder + " to " + f[0])
        ftp.retrbinary("RETR " + args.folder + "/" + f[1], fd.write)
        print("#pragma message: Download of " + f[1] + " from Swing Succes!")
    except IOError:
        print("#pragma message: Fail to open file " + args.file)
    except:
        os.remove(args.file)
        print("#pragma message: Download of " + f[1] + " from Swing Failed!")
    else:
        fd.close()

elif args.command == 'download_dir':
    # Split filename and path
    files = parrot_utils.execute_command(tn, 'find ' + FTP_ROOT + args.folder + ' -name \'*.*\'', prompt=PROMPT)
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
            print("#pragma message: Download of " + f + " from Swing Failed!")
        else:
            fd.close()
    print("#pragma message: End download of folder " + args.folder + " from Swing")

elif args.command == 'rm_dir':
    # Split filename and path
    print("Deleting folder " + FTP_ROOT + args.folder + " from Swing")
    print(parrot_utils.execute_command(tn, 'rm -r ' + FTP_ROOT + args.folder, prompt=PROMPT))



# Close the telnet and python script
parrot_utils.disconnect(tn, ftp)
exit(0)
