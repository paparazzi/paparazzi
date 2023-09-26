#!/usr/bin/python3
import sys
from collections import namedtuple
from pyproj import CRS, Proj
import re
from os.path import basename

LEAP_SECONDS = 18
Point = namedtuple("Point", ["time", "lat", "lon", "alt"])


def time_from_tow(tow):
    utc_s = (tow/1000 - LEAP_SECONDS) % (3600*24)
    h = int(utc_s // 3600)
    m = int((utc_s//60) % 60)
    s = round(utc_s % 60)
    utc_time=f"{h:02d}:{m:02d}:{s:.3f}"
    return utc_time


def date_from_filename(logfile):
    m = re.match("^(?:.*/)?(\d+)_(\d+)_(\d+)__(\d+)_(\d+)_(\d+)(?:_SD)?.data$", logfile)
    if m is not None:
        year, month ,day, h,m,s = m.groups()
        date = f"20{year}-{month}-{day}"
        #time = f"{h}:{m}:{s}"
        return date


def main(logfile):
    """
    extract position data from log. Use GPS time of week.
    """
    with open(logfile, 'r') as log:
        data = {}
        projs = {}
        start_tow = None

        for line in log.readlines():
            args = line.strip().split()
            ac_id = args[1]
            if args[2] == "INS_REF":
                lat = float(args[6]) / 1e7
                lon = float(args[7]) / 1e7
                crs = CRS(f"+proj=ortho +lat_0={lat} +lon_0={lon}")
                projs[ac_id] = Proj.from_crs(crs, "EPSG:4326")

            if args[2] == "GPS_INT":
                tow = int(args[15])
                t = float(args[0])
                start_tow = tow - t

            if args[2] == "ROTORCRAFT_FP" and ac_id in projs and start_tow is not None:
                east = int(args[3])*0.0039063
                north = int(args[4])*0.0039063
                up = int(args[5])*0.0039063
                lat, lon = projs[ac_id].transform(east, north)
                tow = float(args[0]) + start_tow
                utc_time = time_from_tow(tow)
                p = Point(utc_time, lat, lon, up)
                data.setdefault(ac_id, []).append(p)
        for ac_id, points in data.items():
            prefix = f"{sys.argv[2]}_" if len(sys.argv) > 2 else f"{basename(logfile).strip('.data')}__"
            filename = f"{prefix}{ac_id}.csv"
            with open(filename, 'w') as out:
                print(f"created {filename}")
                out.write("time,latitude,longitude,altitude\n")
                date = date_from_filename(logfile)
                for p in points:
                    line = f"{date}T{p.time},{p.lat:.07f},{p.lon:.07f},{p.alt:.01f}\n"
                    out.write(line)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: ./log_extract.py the_log.data [prefix out]")
        exit(1)
    main(sys.argv[1])
