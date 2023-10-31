#!/bin/bash

export ROS1_INSTALL_PATH="/opt/ros/noetic"
export ROS2_INSTALL_PATH="/opt/ros2_humble/install"

#echo "🐢 Sourcing ROS Noetic..."
#source /opt/ros/noetic/setup.bash

#echo "🐢 Sourcing ROS2 Humble..."
#source /opt/ros2_humble/install/setup.bash

echo "👍 Finished environment setup."
echo ""

echo ""
echo "====================================================================="
echo "🚀 To launch the ROS1-ROS2 Bridge: "
echo " ./run_bridge.bash"
echo "====================================================================="
echo ""

if [ -z ${@+x} ]; then
    exec bash
else 
    exec bash -c "$@"
fi
