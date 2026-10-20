"""Launch sequential Nav2 goals and camera-based YOLO detection."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('assign4_bringup'))
    config_file = str(package_share / 'config' / 'autonomy.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('navigation', default_value='true'),
        DeclareLaunchArgument('yolo', default_value='true'),
        DeclareLaunchArgument('show_window', default_value='false'),
        DeclareLaunchArgument('model_path', default_value='yolov8n.pt'),
        DeclareLaunchArgument(
            'image_topic', default_value='/oakd/rgb/preview/image_raw'),
        Node(
            package='assign4_bringup',
            executable='waypoint_navigator.py',
            name='waypoint_navigator',
            output='screen',
            parameters=[config_file],
            condition=IfCondition(LaunchConfiguration('navigation')),
        ),
        Node(
            package='assign4_bringup',
            executable='yolo_detector.py',
            name='yolo_detector',
            output='screen',
            parameters=[
                config_file,
                {
                    'model_path': LaunchConfiguration('model_path'),
                    'image_topic': LaunchConfiguration('image_topic'),
                    'show_window': LaunchConfiguration('show_window'),
                },
            ],
            condition=IfCondition(LaunchConfiguration('yolo')),
        ),
    ])

