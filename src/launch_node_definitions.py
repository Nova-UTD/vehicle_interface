from os import name, path, environ

from pathlib import Path
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory

camera = Node(
    package='camera',
    executable='camera_node'
)

clock = Node(
    package='clock',
    executable='clock_node'
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
    arguments=['/vehicle_interface/data/hail_bopp.urdf']
)

joystick_microsoft = Node(
    package='joy_linux',
    executable='joy_linux_node',
    parameters=[
    {"dev":"/dev/input/by-id/usb-©Microsoft_Corporation_Controller_061ABA4-joystick"}
    ]
)

# os-991234567890.local
ouster_ros_pkg_dir = get_package_share_directory('ouster_ros')
# use the community_driver_config.yaml by default
default_params_file = Path(ouster_ros_pkg_dir) / 'config' / 'community_driver_config.yaml'
params_file = LaunchConfiguration('params_file')
params_file_arg = DeclareLaunchArgument('params_file',
                                        default_value=str(
                                            default_params_file),
                                        description='name or path to the parameters file to use.')

driver_launch_file_path = Path(ouster_ros_pkg_dir) / 'launch' / 'driver.launch.py'
lidar_ouster_driver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([str(driver_launch_file_path)]),
    launch_arguments={
        'params_file': params_file,
        'ouster_ns': '',
        'os_driver_name': 'ouster_driver',
        'viz': 'True',
        'rviz_config': './install/ouster_ros/share/ouster_ros/config/community_driver.rviz'
    }.items()
)

lidar_driver_right = Node(
    package='velodyne_driver',
    executable='velodyne_driver_node',
    parameters=[
        "/vehicle_interface/data/lidar/lidar_driver_right.param.yaml"],
    namespace='velo_right'
)

lidar_driver_left = Node(
    package='velodyne_driver',
    executable='velodyne_driver_node',
    parameters=[
        "/vehicle_interface/data/lidar/lidar_driver_left.param.yaml"],
    namespace='velo_left'
)

lidar_pointcloud_left = Node(
    package='velodyne_pointcloud',
    executable='velodyne_transform_node',
    parameters=[
        "/vehicle_interface/data/lidar/lidar_pointcloud_left.param.yaml"],
    namespace='velo_left'
)

lidar_pointcloud_right = Node(
    package='velodyne_pointcloud',
    executable='velodyne_transform_node',
    parameters=[
        "/vehicle_interface/data/lidar/lidar_pointcloud_right.param.yaml"],
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
    arguments=['-d' + '/vehicle_interface/data/real_world.rviz'],
    respawn=True
)
