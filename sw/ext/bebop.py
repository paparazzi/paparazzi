#!/usr/bin/env python

from __future__ import print_function
import re
import argparse
import socket
import telnetlib
import os
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

# Check if custom wifi_setup script is installed
def check_wifi_setup():
    check_wifi = execute_command('grep "static_ip_address_base" /bin/wifi_setup.sh')
    if "static_ip_address_base" in check_wifi:
        return True
    else:
        return False

# Check the filesystem
def check_filesystem():
    return execute_command('df -h')

# Reboot the drone
def bebop_reboot():
    execute_command('reboot')

# Install the vision framework
def bebop_install_vision():
    print('Uploading GST')
    ftp.storbinary("STOR arm_light.tgz", file("bin/arm_light.tgz", "rb"))
    print(execute_command("cd /data/video && tar -xzf arm_light.tgz"))
    print(execute_command("rm -rf /data/video/arm_light.tgz"))
    print('Now Starting Vision')
    bebop_start_vision()

# Remove the vision framework
def bebop_remove_vision():
    execute_command("rm -rf /opt/arm")
    execute_command("rm -rf /lib/dsp")
    execute_command("rm -rf /data/video/opt")

# Start the vision framework
def bebop_start_vision():
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

# Install autoboot script
def bebop_install_autoboot():
    print('Uploading autoboot script')
    ftp.storbinary("STOR check_update.sh", file("check_update.sh", "rb"))
    print(execute_command("mv /data/video/check_update.sh /bin/check_update.sh"))
    print(execute_command("chmod 777 /bin/check_update.sh"))

# Install network script
def bebop_install_network_script():
    print('Uploading Wifi script')
    ftp.storbinary("STOR wifi_setup.sh", file("wifi_setup.sh", "rb"))
    print(execute_command("mv /data/video/wifi_setup.sh /bin/wifi_setup.sh"))
    print(execute_command("chmod 777 /bin/wifi_setup.sh"))

# Set network SSID
def bebop_set_ssid(name):
    write_to_config('ssid_single_player', name)
    print('The network ID (SSID) of the Bebop is changed to ' + name)

# Set IP address
def bebop_set_ip_address(address):
    splitted_ip = address.split(".")
    write_to_config('static_ip_address_base', splitted_ip[0] + '.' + splitted_ip[1] + '.' + splitted_ip[2] + '.')
    write_to_config('static_ip_address_probe', splitted_ip[3])
    print('The IP Address of the Bebop is changed to ' + address)

# Set wifi mode (0: master, 1: ad-hoc, 2: managed, *: master)
def bebop_set_wifi_mode(mode):
    modes = { 'master' : '0', 'ad-hoc' : '1', 'managed' : '2' }
    try:
        val = modes[mode]
    except:
        print('Unexpected wifi mode, setting to master (default)')
        val = modes['master']
    write_to_config('wifi_mode', val)
    print('The Wifi mode of the Bebop is changed to ' + mode + ' (' + val + ')')

def bebop_status():
    config_ini = execute_command('cat /data/config.ini')

    print('======================== Bebop Status ========================')
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
    if check_vision_running():
        vision_framework += " and running"
    print('Vision framework:\t' + vision_framework)

    # Request the filesystem status
    print('\n======================== Filesystem Status ========================')
    print(check_filesystem())


# Parse the arguments
parser = argparse.ArgumentParser(description='Bebop python helper. Use bebop.py -h for help')
parser.add_argument('--host', metavar='HOST', default='192.168.42.1',
                    help='the ip address of bebop')
subparsers = parser.add_subparsers(title='Command to execute', metavar='command', dest='command')

# All the subcommands and arguments
subparsers.add_parser('status', help='Request the status of the Bebop')
subparsers.add_parser('reboot', help='Reboot the Bebop')
subparsers.add_parser('installvision', help='Install the vision framework')
subparser_upload_gst = subparsers.add_parser('upload_gst_module',
                                         help='Upload, configure and move a gstreamer0.10 module libXXX.so')
subparser_upload_gst.add_argument('file', help='Filename of *.so module')
subparser_upload_and_run = subparsers.add_parser('upload_file_and_run', help='Upload and run software (for instance the Paparazzi autopilot)')
subparser_upload_and_run.add_argument('file', help='Filename of an executable')
subparser_upload_and_run.add_argument('folder', help='Destination subfolder (raw or sdk for Paparazzi autopilot)')
subparser_upload = subparsers.add_parser('upload_file', help='Upload a file to the Bebop')
subparser_upload.add_argument('file', help='Filename')
subparser_upload.add_argument('folder', help='Destination subfolder (base destination folder is /data/video)')
subparser_download = subparsers.add_parser('download_file', help='Download a file from the Bebop')
subparser_download.add_argument('file', help='Filename (with the path on the local machine)')
subparser_download.add_argument('folder', help='Remote subfolder (base folder is /data/video)')
subparser_download_dir = subparsers.add_parser('download_dir', help='Download all files from a folder from the Bebop')
subparser_download_dir.add_argument('dest', help='destination folder (on the local machine)')
subparser_download_dir.add_argument('folder', help='Remote subfolder (base folder is /data/video)')
subparser_rm_dir = subparsers.add_parser('rm_dir', help='Remove a directory and all its files from the Bebop')
subparser_rm_dir.add_argument('folder', help='Remote subfolder (base folder is /data/video)')
subparser_insmod = subparsers.add_parser('insmod', help='Upload and insert kernel module')
subparser_insmod.add_argument('file', help='Filename of *.ko kernel module')
subparsers.add_parser('startvision', help='Start the vision framework')
subparser_start = subparsers.add_parser('start', help='Start a program on the Bebop')
subparser_start.add_argument('program', help='the program to start')
subparser_kill = subparsers.add_parser('kill', help='Kill a program on the Bebop')
subparser_kill.add_argument('program', help='the program to kill')
subparser_networkid = subparsers.add_parser('networkid', help='Set the network ID(SSID) of the Bebop')
subparser_networkid.add_argument('name', help='the new network ID(SSID)')
subparser_ipaddress = subparsers.add_parser('ipaddress', help='Set the IP address of the Bebop')
subparser_ipaddress.add_argument('address', help='the new IP address')
subparser_wifimode = subparsers.add_parser('wifimode', help='Set the Wifi mode the Bebop')
subparser_wifimode.add_argument('mode', help='the new Wifi mode', choices=['master', 'ad-hoc', 'managed'])
subparser_configure_network = subparsers.add_parser('configure_network', help='Configure the network on the Bebop')
subparser_configure_network.add_argument('name', help='the new network ID(SSID)')
subparser_configure_network.add_argument('address', help='the new IP address')
subparser_configure_network.add_argument('mode', help='the new Wifi mode', choices=['master', 'ad-hoc', 'managed'])
subparser_install_autostart = subparsers.add_parser('install_autostart', help='Install custom autostart script and set what to start on boot for the Bebop')
subparser_install_autostart.add_argument('type', choices=['native', 'paparazzi_raw', 'paparazzi_sdk'],
                                 help='what to start on boot')
subparser_autostart = subparsers.add_parser('autostart', help='Set what to start on boot for the Bebop')
subparser_autostart.add_argument('type', choices=['native', 'paparazzi_raw', 'paparazzi_sdk'],
                                 help='what to start on boot')

args = parser.parse_args()

# Connect with telnet and ftp
try:
    tn = telnetlib.Telnet(args.host)
    ftp = FTP(args.host)
    ftp.login()
except:
    print('Could not connect to Bebop (host: ' + args.host + ')')
    exit(2)

# Read until after login
tn.read_until('# ')

# Check the Bebop status
if args.command == 'status':
    bebop_status()

# Reboot the drone
elif args.command == 'reboot':
    bebop_reboot()
    print('The Bebop is rebooting...')

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
    bebop_set_ssid(args.name)

    if raw_input("Shall I restart the Bebop? (y/N) ").lower() == 'y':
        bebop_reboot()

# Change the IP address
elif args.command == 'ipaddress':
    bebop_set_ip_address(args.address)

    if raw_input("Shall I restart the Bebop? (y/N) ").lower() == 'y':
        bebop_reboot()

# Change the wifi mode
elif args.command == 'wifimode':
    bebop_set_wifi_mode(args.mode)

    if raw_input("Shall I restart the Bebop? (y/N) ").lower() == 'y':
        bebop_reboot()

# Install and configure network
elif args.command == 'configure_network':
    config_ini = execute_command('cat /data/config.ini')
    print('=== Current network setup ===')
    print('Network id:\t' + read_from_config('ssid_single_player', config_ini))
    print('Host:\t\t' + args.host + ' (' + read_from_config('static_ip_address_base', config_ini) +
          read_from_config('static_ip_address_probe', config_ini) + ' after boot)')
    print('Mode:\t\t' + read_from_config('wifi_mode', config_ini))
    print('=============================')
    if check_wifi_setup():
        print('Custom Wifi script already installed')
        if raw_input("Shall I reinstall the Wifi script (y/N) ").lower() == 'y':
            bebop_install_network_script()
    else:
        if raw_input("Shall I install custom Wifi script (recommanded) (y/N) ").lower() == 'y':
            bebop_install_network_script()
    bebop_set_ssid(args.name)
    bebop_set_ip_address(args.address)
    bebop_set_wifi_mode(args.mode)
    config_ini = execute_command('cat /data/config.ini')
    print('== New network setup after boot ==')
    print('Network id:\t' + read_from_config('ssid_single_player', config_ini))
    print('Host:\t\t' + read_from_config('static_ip_address_base', config_ini) +
          read_from_config('static_ip_address_probe', config_ini))
    print('Mode:\t\t' + read_from_config('wifi_mode', config_ini))
    print('==================================')

    if raw_input("Shall I restart the Bebop? (y/N) ").lower() == 'y':
        bebop_reboot()

# Install and configure autostart
elif args.command == 'install_autostart':
    if check_autoboot():
        print('Custom autostart script already installed')
        if raw_input("Shall I reinstall the autostart script (y/N) ").lower() == 'y':
            bebop_install_autoboot()
    else:
        bebop_install_autoboot()
    autorun = {'native': '0', 'paparazzi_raw': '1', 'paparazzi_sdk': '2'}
    write_to_config('start_paparazzi', autorun[args.type])
    print('The autostart on boot is changed to ' + args.type)

    if raw_input("Shall I restart the Bebop? (y/N) ").lower() == 'y':
        bebop_reboot()

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
            bebop_remove_vision()
            bebop_install_vision()

    bebop_install_vision()
    print('Vision framework installed')

# Start Vision framework
elif args.command == 'startvision':
    if check_vision_running():
        print('Vision framework already started')
    else:
        if not check_vision_installed():
            print('No vision framework installed')
            if raw_input("Shall I install the vision framework? (y/N) ").lower() == 'y':
                bebop_install_vision()

        if check_vision_installed():
            bebop_start_vision()
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
            if raw_input("Warning: Shall I install the vision framework? (y/N) ").lower() == 'y':
                bebop_install_vision()

        if check_vision_installed():
            bebop_start_vision()
            print('#pragma message: Vision framework started')
    print('#pragma message: Vision Plugin Uploaded and DSP Started.')


elif args.command == 'insmod':
    modfile = split_into_path_and_file(args.file)
    print('Uploading \'' + modfile[1])
    ftp.storbinary("STOR " + modfile[1], file(args.file, "rb"))
    print(execute_command("insmod /data/video/" + modfile[1]))

elif args.command == 'upload_file_and_run':
    # Split filename and path
    f = split_into_path_and_file(args.file)

    print("Kill running " + f[1] + " and make folder " + args.folder)
    execute_command("killall -9 " + f[1])
    sleep(1)
    execute_command("mkdir -p /data/ftp/" + args.folder)
    print('Uploading \'' + f[1] + "\' from " + f[0] + " to " + args.folder)
    ftp.storbinary("STOR " + args.folder + "/" + f[1], file(args.file, "rb"))
    sleep(0.5)
    execute_command("chmod 777 /data/ftp/" + args.folder + "/" + f[1])
    execute_command("/data/ftp/" + args.folder + "/" + f[1] + " > /dev/null 2>&1 &")
    print("#pragma message: Upload and Start of ap.elf to Bebop Succes!")

elif args.command == 'upload_file':
    # Split filename and path
    f = split_into_path_and_file(args.file)

    execute_command("mkdir -p /data/video/" + args.folder)
    print('Uploading \'' + f[1] + "\' from " + f[0] + " to /data/video/" + args.folder)
    ftp.storbinary("STOR " + args.folder + "/" + f[1], file(args.file, "rb"))
    print("#pragma message: Upload of " + f[1] + " to Bebop Succes!")

elif args.command == 'download_file':
    # Split filename and path
    f = split_into_path_and_file(args.file)
    # Open file and download
    try:
        file = open(args.file, 'wb')
        print('Downloading \'' + f[1] + "\' from " + args.folder + " to " + f[0])
        ftp.retrbinary("RETR " + args.folder + "/" + f[1], file.write)
        print("#pragma message: Download of " + f[1] + " from Bebop Succes!")
    except IOError:
        print("#pragma message: Fail to open file " + args.file)
    except:
        os.remove(args.file)
        print("#pragma message: Download of " + f[1] + " from Bebop Failed!")
    else:
        file.close()

elif args.command == 'download_dir':
    # Split filename and path
    files = execute_command('find /data/video/' + args.folder + ' -name \'*.*\'')
    # Create dest dir if needed
    if not os.path.exists(args.dest):
        os.mkdir(args.dest)
    # Open file and download
    for f in files.split():
        file_name = split_into_path_and_file(f)
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
            print("#pragma message: Download of " + f + " from Bebop Failed!")
        else:
            file.close()
    print("#pragma message: End download of folder " + args.folder + " from Bebop")

elif args.command == 'rm_dir':
    # Split filename and path
    print("Deleting folder /data/video/" + args.folder + " from Bebop")
    print(execute_command('rm -r /data/video/' + args.folder))



# Close the telnet and python script
tn.close()
ftp.close()
exit(0)
