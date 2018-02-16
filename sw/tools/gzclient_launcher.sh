#!/bin/sh
# This script sets up the GAZEBO_MODEL_PATH for the gazebo client.
export GAZEBO_MODEL_PATH="$PAPARAZZI_HOME/conf/simulator/gazebo/models:$GAZEBO_MODEL_PATH"
exec gzclient
