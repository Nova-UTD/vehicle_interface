from os import name, path, environ

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory

NAVIGATOR_DIR = "/navigator/"



camera = Node(
    package='camera',
    executable='camera_node'
)

camera_streamer = Node(
    package='web_video_server',
    executable='web_video_server'
)


# I don't think we use this... maybe the old gps?
# but we should keep it for broader vehicle interface...
gps_node = Node(
    package='nmea_navsat_driver',
    executable='nmea_serial_driver'
)

gnss = Node(
    package='gnss',
    executable='gnss_interface_node'
)

hailbopp_epas = Node(
    package='epas',
    executable='epas_node'
)

hailbopp_linear_actuator = Node(
    package='linear_actuator',
    executable='linear_actuator_node'
)

hailbopp_mcu = Node(
    package='mcu_interface',
    executable='mcu_interface_node'
)

hailbopp_urdf_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    arguments=[path.join(NAVIGATOR_DIR, "data", "hail_bopp.urdf")]
)

joy = Node(
    package='joy_linux',
    executable='joy_linux_node',
    parameters=[
    {"dev":"/dev/input/by-id/usb-©Microsoft_Corporation_Controller_061ABA4-joystick"}
    ]
)

lidar_driver_right = Node(
    package='velodyne_driver',
    executable='velodyne_driver_node',
    parameters=[
        "/navigator/param/perception/lidar_driver_right.param.yaml"],
    namespace='velo_right'
)

lidar_driver_left = Node(
    package='velodyne_driver',
    executable='velodyne_driver_node',
    parameters=[
        "/navigator/param/perception/lidar_driver_left.param.yaml"],
    namespace='velo_left'
)

lidar_pointcloud_left = Node(
    package='velodyne_pointcloud',
    executable='velodyne_transform_node',
    parameters=[
        "/navigator/param/perception/lidar_pointcloud_left.param.yaml"],
    namespace='velo_left'
)

lidar_pointcloud_right = Node(
    package='velodyne_pointcloud',
    executable='velodyne_transform_node',
    parameters=[
        "/navigator/param/perception/lidar_pointcloud_right.param.yaml"],
    namespace='velo_right'
)

lidar_processor = Node(
    package='sensor_processing',
    executable='dual_lidar_processing_node'
)

radar_processor = Node(
    package='sensor_processing',
    executable='delphi_esr_radar_processing_node'
)


# Customization
rviz = Node(
    package='rviz2',
    namespace='',
    executable='rviz2',
    name='rviz2',
    arguments=['-d' + '/navigator/data/real_world.rviz'],
    respawn=True
)
