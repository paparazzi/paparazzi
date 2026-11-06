#!/usr/bin/env python3
#
# Copyright (C) 2026 Freek van Tienen <freek.v.tienen@gmail.com>
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
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

"""
Copy the EKF2 estimator of PX4 and the PX4 libraries it needs into this folder.

The EKF2 is only maintained inside PX4-Autopilot, so instead of a submodule
only the needed parts are copied here. The uORB message structs used as data
types by the EKF are generated from the PX4 .msg files.

Usage: sync_px4_ekf2.py [<PX4 commit, branch or tag>] [--px4 <local PX4-Autopilot checkout>]
Without a commit, the one in PX4_VERSION is used again.
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PX4_URL = "https://github.com/PX4/PX4-Autopilot.git"

# (source in PX4, destination here)
COPY = [
    ("src/modules/ekf2/EKF", "EKF"),
    ("src/lib/matrix/matrix", "lib/matrix/matrix"),
    ("src/lib/mathlib", "lib/mathlib"),
    ("src/lib/geo", "lib/geo"),
    ("src/lib/lat_lon_alt", "lib/lat_lon_alt"),
    ("src/lib/world_magnetic_model", "lib/world_magnetic_model"),
    ("src/lib/atmosphere", "lib/atmosphere"),
    ("src/lib/ringbuffer", "lib/ringbuffer"),
    ("src/lib/gnss/gnss_checks.cpp", "lib/gnss/gnss_checks.cpp"),
    ("src/lib/gnss/gnss_checks.hpp", "lib/gnss/gnss_checks.hpp"),
    ("LICENSE", "LICENSE"),
]

# Not needed for building the estimator
SKIP_DIRS = {"test", "tests", "documentation", "tuning_tools", "ekf_derivation_tests"}
SKIP_FILE = re.compile(r"(Test|_test|test_)\w*\.(cpp|hpp|h|py)$|^CMakeLists\.txt$|^Kconfig$|\.(py|md|yaml|json|ipynb)$")
KEEP_PY_DIRS = ("generated",)  # generated equations of the derivation are needed, the derivation itself not

# Parameter definitions (defaults) and the EKF2 module mapping them to the EKF parameters
PARAM_FILES = ["src/modules/ekf2/EKF2.cpp", "src/modules/ekf2/*.yaml",
               "src/modules/sensors/vehicle_gps_position/module.yaml"]
PARAM_PREFIX = "INS_EKF2_"  # prefix of the Paparazzi defines

# uORB messages used as data types by the EKF and the GNSS checks
MSGS = ["EstimatorAidSource1d", "EstimatorAidSource2d", "EstimatorAidSource3d", "VehicleGnss"]

MSG_TYPES = {
    "uint64": "uint64_t", "int64": "int64_t", "uint32": "uint32_t", "int32": "int32_t",
    "uint16": "uint16_t", "int16": "int16_t", "uint8": "uint8_t", "int8": "int8_t",
    "float32": "float", "float64": "double", "bool": "bool", "char": "char",
}


def snake(name):
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


def run(cmd, cwd=None):
    subprocess.check_call(cmd, cwd=cwd)


def checkout(ref, workdir):
    """Sparse checkout of the needed PX4 folders"""
    px4 = os.path.join(workdir, "PX4-Autopilot")
    run(["git", "clone", "--quiet", "--filter=blob:none", "--no-checkout", PX4_URL, px4])
    run(["git", "sparse-checkout", "set", "--no-cone"] + ["/" + s for s, _ in COPY] + ["/msg"] +
        ["/" + s for s in PARAM_FILES], cwd=px4)
    run(["git", "checkout", "--quiet", ref], cwd=px4)
    return px4


def copy_tree(src, dst):
    if os.path.isfile(src):
        shutil.copy2(src, dst)
        return
    for root, dirs, files in os.walk(src):
        rel = os.path.relpath(root, src)
        parts = rel.split(os.sep)
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        in_python = "python" in parts or os.path.basename(root) == "python"
        for f in files:
            if in_python or "derivation" in parts:
                # only keep the generated equations
                if not any(k in parts for k in KEEP_PY_DIRS) or not f.endswith((".h", ".hpp")):
                    continue
            elif SKIP_FILE.search(f):
                continue
            os.makedirs(os.path.join(dst, rel), exist_ok=True)
            shutil.copy2(os.path.join(root, f), os.path.join(dst, rel, f))


def gen_msg(px4, name, out_dir, done=None):
    """Generate the uORB struct for a .msg file (only the data and constants, no uORB metadata)"""
    done = set() if done is None else done
    if name in done:
        return
    done.add(name)
    includes, fields, consts = [], [], []
    for line in open(os.path.join(px4, "msg", name + ".msg")):
        line = line.split("#", 1)[0].strip()
        c = re.match(r"(\w+)\s+(\w+)\s*=\s*(\S+)$", line)
        if c:
            consts.append("\tstatic constexpr %s %s = %s;" % (MSG_TYPES[c.group(1)], c.group(2), c.group(3)))
            continue
        m = re.match(r"(\w+)(\[(\d+)\])?\s+(\w+)$", line)
        if not m:
            continue
        if m.group(1) in MSG_TYPES:
            typ = MSG_TYPES[m.group(1)]
        else:
            # nested message
            gen_msg(px4, m.group(1), out_dir, done)
            includes.append('#include "%s.h"' % snake(m.group(1)))
            typ = snake(m.group(1)) + "_s"
        fields.append("\t%s %s%s;" % (typ, m.group(4), "[%s]" % m.group(3) if m.group(3) else ""))
    s = snake(name)
    hdr = ("/* Generated by sync_px4_ekf2.py from PX4 msg/%s.msg, do not edit */\n\n"
           "#pragma once\n\n#include <stdint.h>\n%s\n"
           "struct %s_s {\n%s\n%s};\n" % (name, "".join(i + "\n" for i in includes), s,
                                            "\n".join(fields), "".join("\n" + c for c in consts) + "\n" if consts else ""))
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, s + ".h"), "w") as f:
        f.write(hdr)


def read_param_defaults(px4):
    """PX4 parameter definitions (name -> definition) of the EKF2 and GNSS checks"""
    import glob
    params = {}
    for pattern in PARAM_FILES[1:]:
        for f in sorted(glob.glob(os.path.join(px4, pattern))):
            for group in yaml.safe_load(open(f)).get("parameters", []):
                for name, d in group.get("definitions", {}).items():
                    if "${" not in name and "default" in d:
                        params[name] = d
    return params


def c_value(d):
    v = d["default"]
    if d.get("type") == "float":
        v = repr(float(v))
        return v + "f"
    return str(int(v))


def gen_params(px4, out_file):
    """Generate the defines with the PX4 defaults of all parameters, and the function giving them to the EKF.
    The mapping of the parameters to the EKF is read from the PX4 EKF2 module (EKF2.cpp)."""
    params = read_param_defaults(px4)

    # parameters mapped to the EKF parameters in the constructor of the EKF2 module
    src = open(os.path.join(px4, PARAM_FILES[0])).read()
    start = src.index("_params(_ekf.getParamHandle())")
    body = src[start:src.index("\n{", start)]
    assigns = []
    for line in body.splitlines():
        line = line.strip()
        m = re.match(r"_param_(\w+)\(_params->(.+)\),?$", line)
        if m:
            name = m.group(1).upper()
            if name in params:
                assigns.append("  p->%s = %s%s;" % (m.group(2), PARAM_PREFIX, name))
            else:
                assigns.append("  // %s: no default in PX4" % name)
        elif re.match(r"#\s*(if|endif|else|elif)\b", line):
            assigns.append(re.sub(r"^#\s*", "#", line))

    out = ["/* Generated by sync_px4_ekf2.py from the PX4 parameter definitions, do not edit.",
           " * Each PX4 parameter has a define with its PX4 default, which can be set in the airframe:",
           " * EKF2_<name> is %s<name>, the other parameters (GNSS_<name>) are %s<parameter>. */"
           % (PARAM_PREFIX, PARAM_PREFIX),
           "", "#pragma once", ""]
    for name in sorted(params):
        d = params[name]
        short = d.get("description", {}).get("short", "").strip().replace("*/", "")
        unit = d.get("unit")
        out.append("/** %s: %s%s */" % (name, short, " (%s)" % unit if unit else ""))
        out.append("#ifndef %s%s" % (PARAM_PREFIX, re.sub(r"^(EKF2_)", "", name)))
        out.append("#define %s%s %s" % (PARAM_PREFIX, re.sub(r"^(EKF2_)", "", name), c_value(d)))
        out.append("#endif")
    out += ["", "/** Give the parameters to the EKF, as the PX4 EKF2 module does */",
            "static inline void ins_ekf2_set_px4_params(estimator::parameters *p)", "{"]
    out += [a.replace(PARAM_PREFIX + "EKF2_", PARAM_PREFIX) for a in assigns]
    out += ["}", ""]
    with open(out_file, "w") as f:
        f.write("\n".join(out))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("ref", nargs="?", help="PX4 commit, branch or tag (default: PX4_VERSION)")
    parser.add_argument("--px4", help="use a local PX4-Autopilot checkout instead of cloning")
    args = parser.parse_args()

    version_file = os.path.join(HERE, "PX4_VERSION")
    ref = args.ref or open(version_file).read().split()[0]

    with tempfile.TemporaryDirectory() as tmp:
        px4 = args.px4 or checkout(ref, tmp)
        commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=px4, text=True).strip()

        for _, dst in COPY:
            path = os.path.join(HERE, dst.split("/gnss_checks")[0])
            if os.path.isdir(path):
                shutil.rmtree(path)
            elif os.path.isfile(path):
                os.remove(path)
        shutil.rmtree(os.path.join(HERE, "generated"), ignore_errors=True)

        for src, dst in COPY:
            os.makedirs(os.path.dirname(os.path.join(HERE, dst)) or HERE, exist_ok=True)
            copy_tree(os.path.join(px4, src), os.path.join(HERE, dst))
        for msg in MSGS:
            gen_msg(px4, msg, os.path.join(HERE, "generated", "uORB", "topics"))
        gen_params(px4, os.path.join(HERE, "generated", "ekf2_params.h"))

    with open(version_file, "w") as f:
        f.write(commit + "\n")
    print("PX4 EKF2 synced to %s" % commit)


if __name__ == "__main__":
    sys.exit(main())
