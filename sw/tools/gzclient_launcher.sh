#!/bin/sh
# This script sets up the resource paths and launches the Gazebo Harmonic GUI client.
export GZ_SIM_RESOURCE_PATH="$PAPARAZZI_HOME/conf/simulator/gazebo/models:$GZ_SIM_RESOURCE_PATH"
export GZ_SIM_RESOURCE_PATH="$PAPARAZZI_HOME/sw/ext/tudelft_gazebo_models/models:$GZ_SIM_RESOURCE_PATH"
exec gz sim -g

