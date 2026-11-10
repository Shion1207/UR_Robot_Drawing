"""Speech-to-text and ordered semantic Nav2 routes, independent of simulation startup."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('assign7_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('speech', default_value='true'),
        DeclareLaunchArgument('audio_device', default_value='pulse'),
        DeclareLaunchArgument('audio_file', default_value=''),
        DeclareLaunchArgument('model_path', default_value=str(share/'models/whisper-base')),
        Node(package='assign7_bringup', executable='voice_navigation_node.py',
             name='voice_navigation_node', output='screen', parameters=[{'use_sim_time': True}]),
        Node(package='assign7_bringup', executable='speech_recognition_node.py',
             name='speech_recognition_node', output='screen',
             parameters=[{'use_sim_time': False, 'audio_device': ParameterValue(LaunchConfiguration('audio_device'), value_type=str),
                          'audio_file': ParameterValue(LaunchConfiguration('audio_file'), value_type=str),
                          'model_path': LaunchConfiguration('model_path')}],
             condition=IfCondition(LaunchConfiguration('speech'))),
    ])
