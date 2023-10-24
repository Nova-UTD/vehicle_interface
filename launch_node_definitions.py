from os import name, path, environ

from pathlib import Path
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, EmitEvent, LogInfo
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import launch
import lifecycle_msgs.msg
from launch_ros.actions import LifecycleNode
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition

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

# OUSTER ######################################################################
lidar_ouster_driver = LifecycleNode(
        package='ouster_ros',
        executable='os_driver',
        name='os_driver',
        namespace='ouster',
        parameters=['/vehicle_interface/data/lidar/ouster_params.yaml'],
        output='screen',
    )

sensor_configure_event = EmitEvent(
    event=ChangeState(
        lifecycle_node_matcher=matches_action(lidar_ouster_driver),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
    )
)

sensor_activate_event = RegisterEventHandler(
    OnStateTransition(
        target_lifecycle_node=lidar_ouster_driver, goal_state='inactive',
        entities=[
            LogInfo(msg="os_driver activating..."),
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(lidar_ouster_driver),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
            )),
        ],
        handle_once=True
    )
)

sensor_finalized_event = RegisterEventHandler(
    OnStateTransition(
        target_lifecycle_node=lidar_ouster_driver, goal_state='finalized',
        entities=[
            LogInfo(
                msg="Failed to communicate with the sensor in a timely manner."),
            EmitEvent(event=launch.events.Shutdown(
                reason="Couldn't communicate with sensor"))
        ],
    )
)
###############################################################################

lidar_ouster_processor = Node(
    package='lidar',
    executable='lidar_ouster_processing_node'
)

lidar_velodyne_driver_right = Node(
    package='velodyne_driver',
    executable='velodyne_driver_node',
    parameters=[
        "/vehicle_interface/data/lidar/lidar_driver_right.param.yaml"],
    namespace='velo_right'
)

lidar_velodyne_driver_left = Node(
    package='velodyne_driver',
    executable='velodyne_driver_node',
    parameters=[
        "/vehicle_interface/data/lidar/lidar_driver_left.param.yaml"],
    namespace='velo_left'
)

lidar_velodyne_pointcloud_left = Node(
    package='velodyne_pointcloud',
    executable='velodyne_transform_node',
    parameters=[
        "/vehicle_interface/data/lidar/lidar_pointcloud_left.param.yaml"],
    namespace='velo_left'
)

lidar_velodyne_pointcloud_right = Node(
    package='velodyne_pointcloud',
    executable='velodyne_transform_node',
    parameters=[
        "/vehicle_interface/data/lidar/lidar_pointcloud_right.param.yaml"],
    namespace='velo_right'
)

lidar_velodyne_processor = Node(
    package='lidar',
    executable='dual_lidar_processing_node'
)

radar_processor = Node(
    package='radar',
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
