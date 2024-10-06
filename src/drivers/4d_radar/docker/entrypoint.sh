#!/bin/bash

echo "🐢 Sourcing ROS2 Foxy..."
source /opt/ros/foxy/setup.bash
source /radar/install/setup.bash

echo "👍 Finished environment setup."
echo ""

echo "🥫 Don't forget to set up the network interfaces (on the host system)!"
echo "  For Ethernet interface:"
echo "    sudo ifconfig <interface_name> 192.168.11.17 netmask 255.255.255.0"
echo "  For CAN interface:"
echo "    sudo ip link set up can0 type can bitrate 500000"
echo "    sudo ip link set up can0"

echo ""
echo "====================================================================="
echo "🚀 To launch the Smartmicro radar node: "
echo " ros2 launch umrr_ros2_driver radar.launch.py"
echo ""
echo "💻 To launch rviz: "
echo " rviz2 -d smart_rviz_plugin/config/rviz/recorder.rviz"
echo ""
echo "📄 Note: Ensure the sensor parameters are correctly set in the YAML files."
echo "====================================================================="
echo ""

if [ -z ${@+x} ]; then
    exec bash
else 
    exec bash -c "$@"
fi