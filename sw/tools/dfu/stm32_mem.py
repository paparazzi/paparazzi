#!/usr/bin/env python
#
# stm32_mem.py: STM32 memory access using USB DFU class
# Copyright (C) 2011  Black Sphere Technologies
# Written by Gareth McMullin <gareth@blacksphere.co.nz>
# Modified by Felix Ruess <felix.ruess@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from __future__ import print_function

from time import sleep
import struct
from sys import stdout, argv
from os import path

from optparse import OptionParser

import usb
import dfu
import time
import numpy

APP_ADDRESS = 0x08002000
SECTOR_SIZE = 2048

CMD_GETCOMMANDS = 0x00
CMD_SETADDRESSPOINTER = 0x21
CMD_ERASE = 0x41


def stm32_erase(dev, addr):
    erase_cmd = struct.pack("<BL", CMD_ERASE, addr)
    dev.download(0, erase_cmd)
    while True:
        status = dev.get_status()
        if status.bState == dfu.STATE_DFU_DOWNLOAD_BUSY:
            sleep(status.bwPollTimeout / 1000.0)
        if status.bState == dfu.STATE_DFU_DOWNLOAD_IDLE:
            break


def stm32_write(dev, data, crc):
    dev.download(2+crc, data)
    while True:
        status = dev.get_status()
        if status.bState == dfu.STATE_DFU_DOWNLOAD_BUSY:
            sleep(status.bwPollTimeout / 1000.0)
        if status.bState == dfu.STATE_DFU_DOWNLOAD_IDLE:
            break


def stm32_manifest(dev):
    dev.download(0, "")
    while True:
        try:
            status = dev.get_status()
        except:
            return
        sleep(status.bwPollTimeout / 1000.0)
        if status.bState == dfu.STATE_DFU_MANIFEST:
            break


def print_copyright():
    print("")
    print("USB Device Firmware Upgrade - Host Utility -- version 1.3")
    print("Copyright (C) 2011  Black Sphere Technologies")
    print("Copyright (C) 2012  Transition Robotics Inc.")
    print("License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>")
    print("")


def init_progress_bar():
    max_symbols = 50
    print("[0%" + "=" * int(max_symbols / 2 - 4) + "50%" + "=" * int(max_symbols / 2 - 4) + "100%]")
    print(" ", end="")
    update_progress_bar.count = 0
    update_progress_bar.symbol_limit = max_symbols


def update_progress_bar(completed, total):
    if completed and total:
        percent = 100 * (float(completed) / float(total))
        if percent >= (update_progress_bar.count + (100.0 / update_progress_bar.symbol_limit)):
            update_progress_bar.count += (100.0 / update_progress_bar.symbol_limit)
            print("#", end="")
        stdout.flush()


if __name__ == "__main__":
    usage = "Usage: %prog [options] [firmware.bin]" + "\n" + "Run %prog --help to list the options."
    parser = OptionParser(usage, version='%prog version 1.3')
    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose")
    parser.add_option("-l", "--list", action="store_true", dest="list_only",
                      help="Only list currently connected DFU devices without actually flashing.")
    parser.add_option("--product", type="choice", choices=["any", "Lisa/Lia"],
                      action="store", default="Lisa/Lia",
                      help="only upload to device where idProduct contains PRODUCT\n"
                           "choices: (any, Lisa/Lia), default: Lisa/Lia")
    parser.add_option("--addr", type="int", action="store", dest="addr", default=APP_ADDRESS,
                      help="Upload start address (default: 0x08002000)")
    parser.add_option("-n", "--dry-run", action="store_true",
                      help="Alias for --list.")
    (options, args) = parser.parse_args()

    if options.dry_run:
        options.list_only = True

    if not options.list_only:
        if len(args) != 1:
            parser.error("incorrect number of arguments")
        else:
            if path.isfile(args[0]):
                binfile = args[0]
            else:
                parser.error("Binary file " + args[0] + " not found")

    if options.verbose:
        print_copyright()

    for i in range(1, 60):
        devs = dfu.finddevs()
        if not devs:
            print('.', end="")
            stdout.flush()
            time.sleep(0.5)
        else:
            break
    print("")
    if not devs:
        print("No DFU devices found!")
        exit(1)
    elif options.verbose or options.list_only:
        print("Found %i DFU devices." % len(devs))

    valid_manufacturers = []
    valid_manufacturers.append("Transition Robotics Inc.")
    valid_manufacturers.append("STMicroelectronics")
    valid_manufacturers.append("Black Sphere Technologies")
    valid_manufacturers.append("TUDelft MavLab. 2012->13")
    valid_manufacturers.append("1 BIT SQUARED")
    valid_manufacturers.append("S.Krukowski")
    valid_manufacturers.append("Paparazzi UAV")

    # list of tuples with possible stm32 (autopilot) devices
    stm32devs = []

    for dev in devs:
        try:
            dfudev = dfu.dfu_device(*dev)
        except:
            if options.verbose:
                print("Could not open DFU device %s ID %04x:%04x "
                      "Maybe the OS driver is claiming it?" %
                      (dev[0].filename, dev[0].idVendor, dev[0].idProduct))
            continue
        try:
            man = dfudev.handle.getString(dfudev.dev.iManufacturer, 30)
            product = dfudev.handle.getString(dfudev.dev.iProduct, 30)
            serial = dfudev.handle.getString(dfudev.dev.iSerialNumber, 40)
        except Exception as e:
            print("Whoops... could not get device description.")
            print("Exception:", e)
            continue

        if options.verbose or options.list_only:
            print("Found DFU device %s: ID %04x:%04x %s - %s - %s" %
                  (dfudev.dev.filename, dfudev.dev.idVendor,
                   dfudev.dev.idProduct, man, product, serial))

        if man in valid_manufacturers:
            if options.product == "any":
                stm32devs.append((dfudev, man, product, serial))
            elif options.product == "Lisa/Lia":
                if "Lisa/M" in product or "Lia" in product or "Fireswarm" in product:
                    stm32devs.append((dfudev, man, product, serial))

    if not stm32devs:
        print("Could not find STM32 (autopilot) device.")
        exit(1)

    if len(stm32devs) > 1:
        print("Warning: Found more than one potential board to flash.")
        for (d, m, p, s) in stm32devs:
            print("Found possible STM32 (autopilot) device %s: ID %04x:%04x %s - %s - %s" %
                  (d.dev.filename, d.dev.idVendor, d.dev.idProduct, m, p, s))

    # use first potential board as target
    (target, m, p, s) = stm32devs[0]
    print("Using device %s: ID %04x:%04x %s - %s - %s" % (target.dev.filename,
                                                          target.dev.idVendor,
                                                          target.dev.idProduct,
                                                          m, p, s))

    # if just listing available devices, exit now
    if options.list_only:
        print("Done.")
        exit(0)

    try:
        state = target.get_state()
    except:
        print("Failed to read device state! Assuming APP_IDLE")
        state = dfu.STATE_APP_IDLE
    if state == dfu.STATE_APP_IDLE:
        target.detach()
        print("Run again to upgrade firmware.")
        exit(0)

    target.make_idle()

    try:
        binf = open(binfile, "rb").read()
    except:
        print("Could not open binary file.")
        raise

    # Get the file length for progress bar
    bin_length = len(binf)

    #addr = APP_ADDRESS
    addr = options.addr
    print("Programming memory from 0x%08X...\r" % addr)

    use_crc = False
    if "CRC" in product:
        use_crc = True

    init_progress_bar()

    while binf:
        update_progress_bar((addr - options.addr), bin_length)
        stm32_erase(target, addr)

        if use_crc:
            write_block = binf[:(SECTOR_SIZE)]
            write_block_array = numpy.frombuffer(write_block, "uint8")
            crc1 = 0
            crc2 = 0
            for b in write_block_array:
                crc1 += b
                crc2 += crc1 & 0xFF
            crc1 &= 0xFF
            crc2 &= 0xFF
            write_block += struct.pack('1B', crc1)
            write_block += struct.pack('1B', crc2)
            stm32_write(target, write_block, 1)
        else:
            stm32_write(target, binf[:SECTOR_SIZE], 0)

        binf = binf[(SECTOR_SIZE):]
        addr += SECTOR_SIZE

    # Need to check all the way to 100% complete
    update_progress_bar((addr - options.addr), bin_length)

    stm32_manifest(target)

    print("\nAll operations complete!")
