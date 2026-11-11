"""Everything needed to use the real robot.

    ros2 launch loquro_bringup robot.launch.py [routine:=squats] [rviz:=true] [bridge:=true] [policy:=<model.onnx>]

- zenoh-bridge-ros2dds: communication with the robot (WiFi). The robot finds it by itself.
- robot_interface: keeps the robot topics declared, sends the PC clock, action /calibrate_imu.
- imu_filter_madgwick: orientation in /imu/data (falls back to accel + gyro if the magnetometer is lost).
- robot_state_publisher: robot model and TF.
- optional: routine_player (routine:=<name>), policy_player (policy:=<path>, placeholder), rviz.
"""
import shutil

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def bridge(context):
    if LaunchConfiguration('bridge').perform(context) != 'true':
        return []
    if shutil.which('zenoh-bridge-ros2dds') is None:
        return [LogInfo(msg='ERROR: zenoh-bridge-ros2dds is not installed (see Software/ros2_ws/README.md). '
                            'Use bridge:=false if it is already running in another machine')]
    config = PathJoinSubstitution([FindPackageShare('loquro_bringup'), 'config', 'zenoh_bridge.json5'])
    return [ExecuteProcess(cmd=['zenoh-bridge-ros2dds', '-c', config], output='screen')]


def optional_nodes(context):
    nodes = []
    routine = LaunchConfiguration('routine').perform(context)
    if routine:
        nodes.append(Node(package='loquro_control', executable='routine_player', output='screen',
                          parameters=[{'routine': routine}]))
    policy = LaunchConfiguration('policy').perform(context)
    if policy:
        nodes.append(Node(package='loquro_control', executable='policy_player', output='screen',
                          parameters=[{'model': policy}]))
    return nodes


def generate_launch_description():
    bringup = FindPackageShare('loquro_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('bridge', default_value='true', description='start zenoh-bridge-ros2dds'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('routine', default_value='', description='routine to play (loquro_control/routines)'),
        DeclareLaunchArgument('policy', default_value='', description='ONNX model for policy_player (placeholder)'),

        OpaqueFunction(function=bridge),

        Node(package='loquro_control', executable='robot_interface', output='screen'),

        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node', name='imu_filter',
             output='screen', parameters=[PathJoinSubstitution([bringup, 'config', 'imu_filter.yaml'])]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('loquro_description'), 'launch', 'display.launch.py'])),
            launch_arguments={'rviz': LaunchConfiguration('rviz')}.items()),

        OpaqueFunction(function=optional_nodes),
    ])
