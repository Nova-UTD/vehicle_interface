from os import name, path, environ
import sys

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory

sys.path.append(path.abspath('/vehicle_interface/'))
from launch_node_definitions import *

def generate_launch_description():

    return LaunchDescription([
        
        hailbopp_urdf_publisher,
        clock,
        # ACTUATION INTERFACE
        hailbopp_epas,
        # hailbopp_mcu,
        # hailbopp_linear_actuator,

        # SENSOR INTERFACE
        joystick_microsoft,
        # #camera,
        gnss,
        # Old lidar (left and right):
        # lidar_driver_left,
        # lidar_pointcloud_left,
        # lidar_driver_right,
        # lidar_pointcloud_right,
        # New lidar(one on the roof rack):
        lidar_ouster_driver, 
        sensor_configure_event, 
        sensor_activate_event, 
        sensor_finalized_event,
        lidar_ouster_processor,
        # radar_processor,

        rviz,
    ])
