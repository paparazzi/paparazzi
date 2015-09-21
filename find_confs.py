#!/usr/bin/env python

from __future__ import print_function

import os
from fnmatch import fnmatch

def find_conf_files(pprz_home, conf_dir, exclude_backups=True):
    conf_files = []
    pattern = "*conf[._-]*xml"
    backup_pattern = "*conf[._-]*xml.20[0-9][0-9]-[01][0-9]-[0-3][0-9]_*"
    excludes = ["conf/%gconf.xml", "conf/conf.xml"]

    for path, subdirs, files in os.walk(conf_dir):
        for name in files:
            if exclude_backups and fnmatch(name, backup_pattern):
                continue
            if fnmatch(name, pattern):
                filepath = os.path.join(path, name)
                entry = os.path.relpath(filepath, pprz_home)
                if not os.path.islink(filepath) and entry not in excludes:
                    conf_files.append(entry)
    return conf_files

if __name__ == "__main__":
    paparazzi_home = os.getenv("PAPARAZZI_HOME", os.path.dirname(os.path.abspath(__file__)))
    # print("PAPARAZZI_HOME=" + paparazzi_home)
    conf_dir = os.path.join(paparazzi_home, "conf")
    # print("conf dir: " + conf_dir)
    conf_files = find_conf_files(paparazzi_home, conf_dir)
    print(" ".join(conf_files))
