#!/bin/sh
# This script sets up environment variables for Gazebo Harmonic with ROS

# Paparazzi paths
export GZ_SIM_RESOURCE_PATH="$PAPARAZZI_HOME/conf/simulator/gazebo/models:$GZ_SIM_RESOURCE_PATH"
export GZ_SIM_RESOURCE_PATH="$PAPARAZZI_HOME/sw/ext/tudelft_gazebo_models/models:$GZ_SIM_RESOURCE_PATH"

# ROS defaults
ROS_SETUP=`locate --regex 'ros/[a-z]*/setup.sh$'`
. $ROS_SETUP

# Launch NPS-Gazebo
exec $PAPARAZZI_HOME/sw/simulator/pprzsim-launch $@
