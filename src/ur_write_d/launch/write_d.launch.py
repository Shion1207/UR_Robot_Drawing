from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    letter_height = LaunchConfiguration("letter_height")
    letter_width = LaunchConfiguration("letter_width")

    # Include the ur_sim_moveit launch file from ur_simulation_gz (restored GUI)
    ur_sim_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_simulation_gz"), "/launch", "/ur_sim_moveit.launch.py"]
        ),
        launch_arguments={
            "ur_type": "ur3e",
            "description_package": "ur_simulation_gz",
            "description_file": "ur_with_position_gain.urdf.xacro",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "letter_height",
                default_value="0.24",
                description="Letter height in metres",
            ),
            DeclareLaunchArgument(
                "letter_width",
                default_value="0.18",
                description="Letter width in metres",
            ),
            ur_sim_moveit_launch,
        ]
    )
