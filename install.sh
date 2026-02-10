#!/bin/bash

USE_VENV=true
BASHRC_SOURCE_VENV=false

# exit on error
set -e

for arg in "$@"
do
  if [ "$arg" = "-h" ] || [ "$arg" = "--help" ]
  then
    echo "Usage: ./install.sh [-n|--no-venv] [-h|---help]"
    echo "  -n, --no-venv   Do not use python virtual environment"
    echo "  -s, --source    Add venv source in ~/.bashrc"
    echo "  -h, --help      Print this help"
    exit 0
  fi

  if [ "$arg" = "-n" ] || [ "$arg" = "--no-venv" ]
  then
    USE_VENV=false
    echo "the venv will not be installed!"
  fi

  if [ "$arg" = "-s" ] || [ "$arg" = "--source" ]
  then
    BASHRC_SOURCE_VENV=true
  fi

done


if [ $VIRTUAL_ENV ]
then
  echo "Cannot create venv from itself! Run 'deactivate' first if you want to recreate the venv."
  USE_VENV = false
  BASHRC_SOURCE_VENV=false
fi


if [ "$USE_VENV" = true ]
then
  sudo apt install -y python3 python3-venv
  python3 setup.py
  source pprzEnv/bin/activate

  if [ "$BASHRC_SOURCE_VENV" = true ]
  then
    echo "venv source will be added to ~/.bashrc"
    echo "source $(pwd)/pprzEnv/bin/activate" >> ~/.bashrc
  fi

else
  sudo apt-get install -f -y python3 python3-pyqt5
fi

python3 ./sw/tools/install.py


