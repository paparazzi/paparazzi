import venv
import os
import sys
import subprocess
import argparse
import shutil

ENV_NAME = 'pprzEnv'


def run(args):
    if args.clean:
        if os.path.exists(ENV_NAME):
            print("Cleaning previous venv.")
            shutil.rmtree(ENV_NAME)
        else:
            print("No previous venv to clean.")

    print("Creating a virtual environment for Paparazzi...")
    venv.create(ENV_NAME, with_pip=True, system_site_packages=args.system)

    # installing requirements
    cmd = [f'./{ENV_NAME}/bin/pip', 'install', '-r' , 'requirements.txt']
    result = subprocess.run(cmd, check=False)
    if result.returncode:
        print("Failed to create venv!")
    else:
        print("venv successfully created.")

    # installing pprzlink
    print("Installing pprzlink...")
    cmd = [f'./{ENV_NAME}/bin/pip', 'install', '-e' , 'sw/ext/pprzlink/lib/v2.0/python']
    result = subprocess.run(cmd, check=False)
    if result.returncode:
        print("Failed to install pprzlink!")
    else:
        print("pprzlink successfully installed.")
    
    if not os.path.exists(f'./{ENV_NAME}/bin/calibrate.py'):
        os.symlink('../../sw/tools/calibration/calibrate.py', f'./{ENV_NAME}/bin/calibrate.py')


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="This script will setup the python environment for Paparazzi")
    parser.add_argument('-s', '--system', action='store_true', help="Use system site packages.")
    parser.add_argument('-c', '--clean', action='store_true', help="Delete the previous virtual environment before recreating it.")
    args = parser.parse_args()

    run(args)
