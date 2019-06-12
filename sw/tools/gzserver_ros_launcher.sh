#!/bin/sh
# This script sets up environment variables for gazebo_ros

# Paparazzi paths
export GAZEBO_MODEL_PATH="$PAPARAZZI_HOME/conf/simulator/gazebo/models:$GAZEBO_MODEL_PATH"
export GAZEBO_MODEL_PATH="$PAPARAZZI_HOME/sw/ext/tudelft_gazebo_models/models:$GAZEBO_MODEL_PATH"

# ROS and Gazebo defaults
ROS_SETUP=`locate --regex 'ros/[a-z]*/setup.sh$'`
GAZEBO_SETUP=`locate --regex 'gazebo/setup.sh$'`
. $ROS_SETUP
. $GAZEBO_SETUP

# Launch NPS-Gazebo
exec $PAPARAZZI_HOME/sw/simulator/pprzsim-launch $@
