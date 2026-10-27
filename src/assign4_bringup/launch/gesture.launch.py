"""Week 6: webcam recognition plus confirmed robot behaviors, without Nav2."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('assign4_bringup'))
    config = str(share / 'config/gesture.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('camera_index', default_value='0'),
        DeclareLaunchArgument('show_window', default_value='true'),
        DeclareLaunchArgument('recognition', default_value='true'),
        DeclareLaunchArgument('behavior', default_value='true'),
        DeclareLaunchArgument('model_path', default_value=str(share / 'models/gesture_recognizer.task')),
        Node(package='assign4_bringup', executable='robot_behavior_node.py',
             name='robot_behavior_node', output='screen', parameters=[config],
             condition=IfCondition(LaunchConfiguration('behavior'))),
        Node(package='assign4_bringup', executable='gesture_recognition_node.py',
             name='gesture_recognition_node', output='screen',
             parameters=[config, {
                 'camera_index': ParameterValue(LaunchConfiguration('camera_index'), value_type=int),
                 'show_window': ParameterValue(LaunchConfiguration('show_window'), value_type=bool),
                 'model_path': LaunchConfiguration('model_path'),
             }], condition=IfCondition(LaunchConfiguration('recognition'))),
    ])
