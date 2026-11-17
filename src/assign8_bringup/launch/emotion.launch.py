from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config=str(Path(get_package_share_directory('assign8_bringup'))/'config/emotion.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('camera_index',default_value='0'),
        DeclareLaunchArgument('show_window',default_value='true'),
        DeclareLaunchArgument('recognition',default_value='true'),
        DeclareLaunchArgument('motion',default_value='true'),
        Node(package='assign8_bringup',executable='emotion_motion_node.py',name='emotion_motion_node',
             parameters=[config],output='screen',condition=IfCondition(LaunchConfiguration('motion'))),
        Node(package='assign8_bringup',executable='emotion_recognition_node.py',name='emotion_recognition_node',
             parameters=[config,{'camera_index':ParameterValue(LaunchConfiguration('camera_index'),value_type=int),
                 'show_window':ParameterValue(LaunchConfiguration('show_window'),value_type=bool)}],
             output='screen',condition=IfCondition(LaunchConfiguration('recognition'))),
    ])
