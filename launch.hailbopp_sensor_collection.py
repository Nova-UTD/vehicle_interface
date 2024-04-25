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

	launch.actions.DeclareLaunchArgument(
            "log_level",
            default_value=["debug"],
            description="Logging level",
      	),

        clock,
        hailbopp_urdf_publisher,

        # SENSOR INTERFACE
        #camera,
        # gnss,
        lidar_ouster_driver, 
        sensor_configure_event, 
        sensor_activate_event, 
        sensor_finalized_event,
        lidar_ouster_processor,
        #lidar_velodyne_driver_left, lidar_velodyne_pointcloud_left,
        #lidar_velodyne_driver_right, lidar_velodyne_pointcloud_right,
        #lidar_velodyne_processor,
        #radar_processor,

        rviz,
    ])
