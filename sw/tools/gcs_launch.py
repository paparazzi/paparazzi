#! /usr/bin/env python3
from optparse import OptionParser, OptionGroup, OptionValueError
import subprocess
from os import getenv, path, execvp

HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../')))
LEGACY_GCS_PATH = path.join(HOME, "sw", "ground_segment", "cockpit", "gcs")


def pprzgcs_help(option, opt, value, parser):
    try:
        cp = subprocess.run(["pprzgcs", "-h"], capture_output=True)
        # trim to relevant output
        lines = cp.stdout.decode().split("\n")[4:]
        options = "\n".join(lines)
        print("PprzGCS options:\n\n" + options)
        exit(0)
    except FileNotFoundError:
        print("PprzGCS not found!")
        exit(1)


def legacy_help(option, opt, value, parser):
    try:
        cp = subprocess.run([LEGACY_GCS_PATH, "--help"], capture_output=True)
        # trim to relevant output
        lines = cp.stdout.decode().split("\n")[1:]
        options = "\n".join(lines)
        print("Legacy GCS options:\n\n" + options)
        exit(0)
    except FileNotFoundError:
        print("Legacy GCS not found!")
        exit(1)


def main():

    usage = "usage: %prog -g <GCS type> -- [GCS arguments]\n" + \
            "Run %prog --help to list the options."
    parser = OptionParser(usage)

    parser.add_option("-g", "--gcs", dest="gcstype",
                      type='choice', choices=['pprzgcs', 'legacy'],
                      action="store", help="GCS type to start: pprgcs or legacy")
    parser.add_option("--pprzgcs_help", dest="pprzgcs_help", action="callback", callback=pprzgcs_help,
                      help="Print help for pprzgcs")
    parser.add_option("--legacy_help", dest="legacy_help", action="callback", callback=legacy_help,
                      help="Print help for legacy GCS")

    (options, args) = parser.parse_args()

    def run_gcs(cmd, args, error_msg):
        try:
            args = [cmd] + args
            print("Running \"" + " ".join(args) + "\"")
            execvp(cmd, args)
        except FileNotFoundError:
            print(error_msg)

    if options.gcstype == "pprzgcs":
        run_gcs("pprzgcs", args, "PprzGCS not found!")
    elif options.gcstype == "legacy":
        run_gcs(LEGACY_GCS_PATH, args, "Legacy GCS not found!")
    elif options.gcstype is None:
        run_gcs("pprzgcs", args, "PprzGCS not found!")
        run_gcs(LEGACY_GCS_PATH, args, "Legacy GCS not found!")


if __name__ == "__main__":
    main()



