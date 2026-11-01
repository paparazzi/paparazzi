#!/bin/sh
# Start the Gazebo GUI with the Paparazzi model paths:
# gzclient for Gazebo Classic, otherwise the gz-sim GUI (connects to the server running in NPS)
if command -v gzclient > /dev/null; then
  export GAZEBO_MODEL_PATH="$PAPARAZZI_HOME/conf/simulator/gazebo/models:$GAZEBO_MODEL_PATH"
  export GAZEBO_MODEL_PATH="$PAPARAZZI_HOME/sw/ext/tudelft_gazebo_models/models:$GAZEBO_MODEL_PATH"
  exec gzclient
else
  export GZ_SIM_RESOURCE_PATH="$PAPARAZZI_HOME/conf/simulator/gazebo/models:$PAPARAZZI_HOME/sw/ext/tudelft_gazebo_models/models:$GZ_SIM_RESOURCE_PATH"
  exec gz sim -g
fi
