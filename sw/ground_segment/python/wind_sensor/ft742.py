#!/usr/bin/env python3
#
# Copyright (C) 2026 TU Delft
#
# This file is part of paparazzi.
#
# paparazzi is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.
#

"""
Read a FT Technologies FT742-SM ultrasonic wind sensor on the ground and
forward the wind (and optionally temperature) to an aircraft with WIND_INFO.

The sensor is polled over serial with `$//,WV?*//` and answers with
`$<talker>,WVP=<speed>,<angle>,<status>*<checksum>`. Temperature is read
with `$//,HT?*//` when --temperature is given.

Usage: ./ft742.py /dev/ttyUSB0 --ac-id 2 [--temperature]
"""

import argparse
import math
import sys
import time
from os import path, getenv

import serial

PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

WIND_QUERY = b"$//,WV?*//\r\n"
TEMP_QUERY = b"$//,HT?*//\r\n"
TEMP_TIMEOUT = 5.0  # s, temperature older than this is not sent

WIND_INFO_HORIZONTAL = 0x01
WIND_INFO_TEMPERATURE = 0x08


def parse_frame(line):
    """Parse a FT742 frame, return a (sentence, fields) tuple or None if invalid."""
    line = line.strip()
    if not line.startswith("$") or "*" not in line:
        return None
    body, checksum = line[1:].split("*", 1)
    try:
        if int(checksum, 16) != _checksum(body):
            return None
    except ValueError:
        return None
    if "," not in body or "=" not in body:
        return None
    sentence, data = body.split(",", 1)[1].split("=", 1)
    return sentence, data.split(",")


def _checksum(body):
    value = 0
    for c in body:
        value ^= ord(c)
    return value


def parse_wind(fields):
    """Return (speed in m/s, angle in deg the wind comes from) or None."""
    try:
        speed_text, angle, status = fields[0], float(fields[1]), int(fields[2])
        # Speed without decimal point is given in tenths of m/s
        speed = float(speed_text) if "." in speed_text else int(speed_text) / 10.
    except (IndexError, ValueError):
        return None
    if status != 0:
        print(f"Sensor status {status}, wind ignored", file=sys.stderr)
        return None
    return speed, angle


def parse_temperature(fields):
    """Return temperature in deg C from a HT frame (humidity, status, temperature) or None."""
    try:
        return float(fields[2])
    except (IndexError, ValueError):
        return None


class FT742:
    def __init__(self, port, baud, ac_id, rate, temperature):
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.2)
        self.ivy = IvyMessagesInterface("ft742")
        self.ac_id = ac_id
        self.period = 1. / rate
        self.read_temperature = temperature
        self.temperature = None
        self.temperature_time = 0.

    def query(self, cmd):
        self.ser.reset_input_buffer()
        self.ser.write(cmd)
        return parse_frame(self.ser.readline().decode("ascii", errors="ignore"))

    def update_temperature(self):
        frame = self.query(TEMP_QUERY)
        if frame is not None and frame[0] == "HT":
            temp = parse_temperature(frame[1])
            if temp is not None:
                self.temperature = temp
                self.temperature_time = time.time()

    def send_wind(self, speed, angle):
        # The sensor gives the direction the wind comes from (0 deg is north, clockwise),
        # WIND_INFO expects the vector the wind blows to, hence the 180 deg.
        # This assumes the sensor north mark is aligned with true north.
        to_dir = math.radians(angle + 180.)
        msg = PprzMessage("datalink", "WIND_INFO")
        msg['ac_id'] = self.ac_id
        msg['flags'] = WIND_INFO_HORIZONTAL
        msg['east'] = speed * math.sin(to_dir)
        msg['north'] = speed * math.cos(to_dir)
        msg['up'] = 0.
        msg['airspeed'] = 0.
        msg['temp'] = -1000.  # unknown
        if self.temperature is not None and time.time() - self.temperature_time < TEMP_TIMEOUT:
            msg['flags'] |= WIND_INFO_TEMPERATURE
            msg['temp'] = self.temperature
        self.ivy.send_raw_datalink(msg)

    def run(self):
        cycle = 0
        missed = 0
        while True:
            start = time.time()
            # Read the temperature once a second, it changes slowly
            if self.read_temperature and cycle % max(1, round(1. / self.period)) == 0:
                self.update_temperature()
            cycle += 1

            frame = self.query(WIND_QUERY)
            wind = parse_wind(frame[1]) if frame is not None and frame[0] == "WVP" else None
            if wind is None:
                missed += 1
                if missed == 10:
                    print("No valid wind from sensor", file=sys.stderr)
            else:
                missed = 0
                self.send_wind(*wind)
                print(f"\rwind {wind[0]:5.1f} m/s from {wind[1]:5.1f} deg, temp {self.temperature}   ", end="")

            time.sleep(max(0., self.period - (time.time() - start)))

    def stop(self):
        self.ivy.shutdown()
        self.ser.close()


def main():
    parser = argparse.ArgumentParser(description="FT742-SM ground wind sensor to WIND_INFO")
    parser.add_argument("port", help="serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--ac-id", type=int, required=True, help="aircraft ID to send the wind to")
    parser.add_argument("--baud", type=int, default=9600, help="serial baud rate")
    parser.add_argument("--rate", type=float, default=10., help="polling rate in Hz (max 10)")
    parser.add_argument("--temperature", action="store_true", help="also read and send the temperature")
    args = parser.parse_args()

    if not 0 < args.rate <= 10.:
        parser.error("--rate must be in (0, 10]")

    try:
        sensor = FT742(args.port, args.baud, args.ac_id, args.rate, args.temperature)
    except serial.SerialException as e:
        sys.exit(f"Serial error: {e}")
    try:
        sensor.run()
    except KeyboardInterrupt:
        pass
    finally:
        sensor.stop()


if __name__ == "__main__":
    main()
