#!/usr/bin/env python

from __future__ import print_function
import re
import argparse
import socket
import telnetlib
from time import sleep
from ftplib import FTP


# Check if IP is valid
def is_ip(address):
    try:
        socket.inet_aton(address)
        ip = True
    except socket.error:
        ip = False
    return ip

# Execute a command
def execute_command(command):
    tn.write(command + '\n')
    return tn.read_until('# ')[len(command) + 2:-4]

# Helper function
def split_into_path_and_file(name):
    if name.count('/') <= 0:
        return ["./", name]
    return name.rsplit('/', 1)

# Read from config.ini
def read_from_config(name, config=''):
    if config == '':
        config = execute_command('cat /data/config.ini')
    search = re.search(name + '[^=]+=[\r\n\t ]([^\r\n\t ]+)', config)
    if search is None:
        return ''
    else:
        return search.group(1)

# Write to config
def write_to_config(name, value):
    if read_from_config(name) == '':
        execute_command('echo "' + name + ' = ' + value + '\" >> /data/config.ini')
    else:
        execute_command('sed -i "s/\(' + name + ' *= *\).*/\\1' + value + '/g" /data/config.ini')

# Check the version
def check_version():
    return execute_command('cat /firmware/version.txt')

# Check what currently is running on the drone
def check_running():
    ps_aux = execute_command('ps')
    running = ""

    if 'program.elf' in ps_aux:
        running += ' Native (program.elf),'
    if 'ap.elf' in ps_aux:
        running += ' Paparazzi (ap.elf),'
    if 'gst-launch' in ps_aux:
        running += ' GStreamer (gst-launch)'
    return running[1:]

# Check if vision framework is installed
def check_vision_installed():
    du_opt = execute_command('du -d 2 /data/video/opt')
    return '/data/video/opt/arm/gst' in du_opt or '/data/video/opt/arm/lib' in du_opt or '/data/video/opt/arm/tidsp-binaries-23.i3.8' in du_opt

# Check if the vision framework is running
def check_vision_running():
    du_opt = execute_command('du -d 2 /opt')
    return '/opt/arm/gst' in du_opt and '/opt/arm/lib' in du_opt and '/opt/arm/tidsp-binaries-23.i3.8' in du_opt

# Check if autoboot is installed
def check_autoboot():
    check_update = execute_command('grep "START_PAPARAZZI" /bin/check_update.sh')
    wifi_setup = execute_command('grep "BASE_ADRESS" /bin/wifi_setup.sh')
    if "START_PAPARAZZI" in check_update and "BASE_ADRESS" in wifi_setup:
        return True
    else:
        return False

# Check the filesystem
def check_filesystem():
    return execute_command('df -h')

# Reboot the drone
def ardrone2_reboot():
    execute_command('reboot')

# Install the vision framework
def ardrone2_install_vision():
    print('Uploading GST')
    ftp.storbinary("STOR arm_light.tgz", file("bin/arm_light.tgz", "rb"))
    print(execute_command("cd /data/video && tar -xzf arm_light.tgz"))
    print(execute_command("rm -rf /data/video/arm_light.tgz"))
    print('Now Starting Vision')
    ardrone2_start_vision()

# Remove the vision framework
def ardrone2_remove_vision():
    execute_command("rm -rf /opt/arm")
    execute_command("rm -rf /lib/dsp")
    execute_command("rm -rf /data/video/opt")

# Start the vision framework
def ardrone2_start_vision():
    # Mount the directories
    execute_command("mkdir -p /opt/arm")
    execute_command("mkdir -p /lib/dsp")
    execute_command("mount --bind /data/video/opt/arm /opt/arm")
    execute_command("mount --bind /data/video/opt/arm/lib/dsp /lib/dsp")
    # Start The DSP programs
    execute_command("kill -9 `pidof program.elf`")
    execute_command("kill -9 `pidof gst-launch-0.10`")
    execute_command("export PATH=/opt/arm/gst/bin:$PATH")
    execute_command("export DSP_PATH=/opt/arm/tidsp-binaries-23.i3.8/")
    execute_command("/bin/dspbridge/cexec.out -T /opt/arm/tidsp-binaries-23.i3.8/baseimage.dof -v")
    execute_command("/bin/dspbridge/dynreg.out -r /opt/arm/tidsp-binaries-23.i3.8/m4venc_sn.dll64P -v")
    # Show result
    execute_command("ls -altr /opt/arm/gst/bin")


def ardrone2_status():
    config_ini = execute_command('cat /data/config.ini')

    print('======================== ARDrone 2 Status ========================')
    print('Version:\t\t' + check_version())
    print('Host:\t\t\t' + args.host + ' (' + read_from_config('static_ip_address_base', config_ini) +
          read_from_config('static_ip_address_probe', config_ini) + ' after boot)')
    print('Currently running:\t' + check_running())
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
    if check_vision_running:
        vision_framework += " and running"
    print('Vision framework:\t' + vision_framework)

    # Request the filesystem status
    print('\n======================== Filesystem Status ========================')
    print(check_filesystem())


# Parse the arguments
parser = argparse.ArgumentParser(description='ARDrone 2 python helper. Use ardrone2.py -h for help')
parser.add_argument('--host', metavar='HOST', default='192.168.1.1',
                    help='the ip address of ardrone2')
subparsers = parser.add_subparsers(title='Command to execute', metavar='command', dest='command')

# All the subcommands and arguments
subparsers.add_parser('status', help='Request the status of the ARDrone 2')
subparsers.add_parser('reboot', help='Reboot the ARDrone 2')
subparsers.add_parser('installvision', help='Install the vision framework')
subparser_upload = subparsers.add_parser('upload_gst_module',
                                         help='Upload, configure and move a gstreamer0.10 module libXXX.so')
subparser_upload.add_argument('file', help='Filename of *.so module')
subparser_paparazzi = subparsers.add_parser('upload_paparazzi', help='Upload paparazzi autopilot software')
subparser_paparazzi.add_argument('file', help='Filename of *.elf executable')
subparser_paparazzi.add_argument('folder', help='Destination Subfolder: raw or sdk')
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
subparser_autostart = subparsers.add_parser('autostart', help='Set what to start on boot for the ARDrone 2')
subparser_autostart.add_argument('type', choices=['native', 'paparazzi_raw', 'paparazzi_sdk'],
                                 help='what to start on boot')

args = parser.parse_args()

# Connect with telnet and ftp
try:
    tn = telnetlib.Telnet(args.host)
    ftp = FTP(args.host)
    ftp.login()
except:
    print('Could not connect to ARDrone 2 (host: ' + args.host + ')')
    exit(2)

# Read until after login
tn.read_until('# ')

# Check the ARDrone 2 status
if args.command == 'status':
    ardrone2_status()

# Reboot the drone
elif args.command == 'reboot':
    ardrone2_reboot()
    print('The ARDrone 2 is rebooting...')

# Kill a program
elif args.command == 'kill':
    execute_command('killall -9 ' + args.program)
    print('Program "' + args.program + '" is now killed')

# Start a program
elif args.command == 'start':
    execute_command(args.start + ' &')
    print('Program "' + args.start + '" is now started')

# Change the network ID
elif args.command == 'networkid':
    write_to_config('ssid_single_player', args.name)
    print('The network ID (SSID) of the ARDrone 2 is changed to ' + args.name)

    if input("Shall I restart the ARDrone 2? (y/N) ").lower() == 'y':
        ardrone2_reboot()

# Change the IP address
elif args.command == 'ipaddress':
    splitted_ip = args.address.split(".")
    write_to_config('static_ip_address_base', splitted_ip[0] + '.' + splitted_ip[1] + '.' + splitted_ip[2] + '.')
    write_to_config('static_ip_address_probe', splitted_ip[3])
    print('The IP Address of the ARDrone 2 is changed to ' + args.address)

    if input("Shall I restart the ARDrone 2? (y/N) ").lower() == 'y':
        ardrone2_reboot()

# Change the autostart
elif args.command == 'autostart':
    autorun = {'native': '0', 'paparazzi_raw': '1', 'paparazzi_sdk': '2'}
    write_to_config('start_paparazzi', autorun[args.type])
    print('The autostart on boot is changed to ' + args.type)

# Install Vision framework
elif args.command == 'installvision':
    if check_vision_installed():
        print('Vision framework already installed')
        if input("Shall I reinstall the vision framework? (y/N) ").lower() == 'y':
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
            if input("Shall I install the vision framework? (y/N) ").lower() == 'y':
                ardrone2_install_vision()

        if check_vision_installed():
            ardrone2_start_vision()
            print('Vision framework started')

elif args.command == 'upload_gst_module':
    print('Uploading ...' + args.file)
    ftp.storbinary("STOR " + args.file, file(args.file, "rb"))
    execute_command("chmod 777 /data/video/" + args.file)
    execute_command("mv /data/video/" + args.file + " /data/video/opt/arm/gst/lib/gstreamer-0.10")
    if check_vision_running():
        print('Info: Vision framework already started')
    else:
        if not check_vision_installed():
            print('Warning: No vision framework installed')
            if input("Warning: Shall I install the vision framework? (y/N) ").lower() == 'y':
                ardrone2_install_vision()

        if check_vision_installed():
            ardrone2_start_vision()
            print('#pragma message: Vision framework started')
    print('#pragma message: Vision Plugin Uploaded and DSP Started.')


elif args.command == 'insmod':
    modfile = split_into_path_and_file(args.file)
    print('Uploading \'' + modfile[1])
    ftp.storbinary("STOR " + modfile[1], file(args.file, "rb"))
    print(execute_command("insmod /data/video/" + modfile[1]))

elif args.command == 'upload_paparazzi':
    # Split filename and path
    f = split_into_path_and_file(args.file)

    print("Kill running " + f[1] + " and make folder " + args.folder)
    execute_command("killall -9 " + f[1])
    sleep(1)
    execute_command("mkdir -p /data/video/" + args.folder)
    print('Uploading \'' + f[1] + "\' from " + f[0] + " to " + args.folder)
    ftp.storbinary("STOR " + args.folder + "/" + f[1], file(args.file, "rb"))
    sleep(0.5)
    execute_command("chmod 777 /data/video/" + args.folder + "/" + f[1])
    execute_command("/data/video/" + args.folder + "/" + f[1] + " > /dev/null 2>&1 &")
    print("#pragma message: Upload and Start of ap.elf to ARDrone2 Succes!")



# Close the telnet and python script
tn.close()
ftp.close()
exit(0)
