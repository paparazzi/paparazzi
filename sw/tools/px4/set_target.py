#!/usr/bin/env python3

from __future__ import print_function
import os
import sys
import serial
import glob
import time

target = sys.argv[1]
firmware_file = sys.argv[2]

print("Target: " + target)
print("Firmware file: " + firmware_file)

# test if pprz cdm is connected
mode = -1
port = ""
try:
    port = "/dev/serial/by-id/usb-Paparazzi_UAV_CDC_Serial_STM32_*"
    if len(glob.glob(port)) > 1:
        print("Warning: multiple Paparazzi cdc devices found. Selecting the first one.")
    port = glob.glob(port)[0]
    ser = serial.Serial(port, timeout=0.5)
    mode = 1
    print("Paparazzi CDC device found at port: " + port)
except (serial.serialutil.SerialException, IndexError):
    print("No Paparazzi CDC device found, looking further.")

if mode == 1:
    if target == "fbw":
        line = "pprz0"
    else:
        line = "pprz1"

    print ("Sending target command to Paparazzi firmware...")
    ser.flush()
    ser.write(line.encode())
    time.sleep(1)
    if target == "fbw":
        try:
            c = ser.read(7)
            print("AP responded with: ")
            print(c)
            if c.decode() == "TIMEOUT":
                print("Error: FBW bootloader TIMEOUT. Power cycle the board and wait between 10 seconds to 20 seconds to retry.")
                sys.exit(1)
            elif c.decode() == "ERROR:1":
                print("Error: FBW firmware handshake failed. Power cycle the board and wait between 10 seconds to 20 seconds to retry.")
                sys.exit(1)
            elif c.decode() == "ERROR:2":
                print("Error: FBW bootloader sync failed. Power cycle the board and wait between 10 seconds to 20 seconds to retry.")
                sys.exit(1)    
            elif c.decode() != "FBWOKOK":
                print("Error: unknown error. Power cycle the board and wait between 10 seconds to 20 seconds to retry.")
                sys.exit(1)
        except serial.serialutil.SerialException:
            pass

    print("Uploading using Paparazzi firmware...")
    if target == "ap":
        print("If the uploading does not start within a few seconds, please replug the usb (power cycle the board).")
    # sys.exit(0)

if mode == -1:  # no pprz cdc was found, look for PX4
    if target == "fbw":
        print("Error: original firmware cannot be used to upload the fbw code. Wait for the PX4 bootloader to exit (takes 5 seconds), or in case this is the first upload; first upload the Paparazzi ap target.")
        sys.exit(1)
    else:
        print("Uploading AP using original PX4 firmware...")
        print("If the uploading does not start within a few seconds, please replug the usb (power cycle the board).")
