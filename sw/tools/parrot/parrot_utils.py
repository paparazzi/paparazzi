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
import socket
import telnetlib
import os
import sys
from ftplib import FTP
import ftplib


# Check if IP is valid
def is_ip(address):
    try:
        socket.inet_aton(address)
        ip = True
    except socket.error:
        ip = False
    return ip

# Helper function
def split_into_path_and_file(name):
    if name.count('/') <= 0:
        return ["./", name]
    return name.rsplit('/', 1)

# Execute a command
def execute_command(tn, command):
    tn.write(command + '\n')
    return tn.read_until('# ')[len(command) + 2:-4]

# Check the version
def check_version(tn, directory):
    return execute_command(tn, 'cat ' + directory + '/version.txt')

# Check what currently is running on the drone
def check_running(tn):
    ps_aux = execute_command(tn, 'ps')
    running = ""

    if 'program.elf' in ps_aux:
        running += ' Native (program.elf),'
    if 'dragon-prog' in ps_aux:
        running += ' Native (dragon-prog),'
    if 'ap.elf' in ps_aux:
        running += ' Paparazzi (ap.elf),'
    if 'gst-launch' in ps_aux:
        running += ' GStreamer (gst-launch)'
    return running[1:]

# Check the filesystem
def check_filesystem(tn):
    return execute_command(tn, 'df -h')

# Reboot the drone
def reboot(tn):
    execute_command(tn, 'reboot')

# Upload ftp and catch memory-full error
def uploadfile(ftp, filename, content):
    try:
        ftp.storbinary("STOR " + filename, content)
    except ftplib.error_temp:
        print("FTP UPLOAD ERROR: Uploading FAILED: Probably your ARDrone memory is full.")
        sys.exit()
    except:
        print("FTP UPLOAD ERROR: Maybe your ARDrone memory is full?", sys.exc_info()[0])
        sys.exit()


# Connect with telnet and ftp, wait until login
def connect(host):
    try:
        tn = telnetlib.Telnet(host, timeout=3)
        ftp = FTP(host)
        ftp.login()
        tn.read_until('# ')
        return tn, ftp
    except:
        print('Could not connect to Parrot UAV (host: ' + host + ')')
        exit(2)

# Close the telnet and ftp
def disconnect(tn, ftp):
    tn.close()
    ftp.close()


