"""Standalone assignment 7 Gazebo + saved map + AMCL + Nav2 + semantic RViz."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(path, args):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)), launch_arguments=args.items())


def simulator(context, share, sim, ign):
    args = f'{share}/worlds/small_room.sdf -r -v 3 -s'
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    if headless:
        args += ' --headless-rendering'
    actions = [GroupAction(scoped=True, actions=[include(ign/'launch/ign_gazebo.launch.py',
               {'gz_args': args, 'on_exit_shutdown': 'true'})])]
    if not headless:
        gui = GroupAction(scoped=True, actions=[include(ign/'launch/ign_gazebo.launch.py',
                {'gz_args': f'-g -v 2 --gui-config {sim}/gui/lite/gui.config', 'on_exit_shutdown': 'false'})])
        actions.append(TimerAction(period=2.0, actions=[gui]))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('assign7_bringup'))
    sim = Path(get_package_share_directory('turtlebot4_ignition_bringup'))
    nav = Path(get_package_share_directory('turtlebot4_navigation'))
    ign = Path(get_package_share_directory('ros_ign_gazebo'))
    create = Path(get_package_share_directory('irobot_create_ignition_bringup'))
    description = Path(get_package_share_directory('turtlebot4_description'))
    create_description = Path(get_package_share_directory('irobot_create_description'))
    create_plugins = Path(get_package_share_directory('irobot_create_ignition_plugins'))
    tb_plugins = Path(get_package_share_directory('turtlebot4_ignition_gui_plugins'))
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', os.pathsep.join(map(str,
            [share/'worlds', sim/'worlds', create/'worlds', description.parent, create_description.parent]))),
        SetEnvironmentVariable('IGN_GUI_PLUGIN_PATH', os.pathsep.join(map(str,[tb_plugins/'lib',create_plugins/'lib']))),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', str(share/'rviz/navigation.rviz')], parameters=[{'use_sim_time':True}],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
        OpaqueFunction(function=simulator, kwargs={'share':share, 'sim':sim, 'ign':ign}),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'], output='screen'),
        GroupAction(scoped=True, actions=[include(sim/'launch/turtlebot4_spawn.launch.py',
            {'namespace':'', 'model':'lite', 'rviz':'false', 'slam':'false', 'localization':'false',
             'nav2':'false', 'use_sim_time':'true', 'x':'0.0', 'y':'0.0', 'z':'0.0', 'yaw':'0.0'})]),
        include(nav/'launch/localization.launch.py', {'namespace':'', 'use_sim_time':'true',
                'params':str(share/'config/localization.yaml'), 'map':str(share/'maps/map.yaml')}),
        include(nav/'launch/nav2.launch.py', {'namespace':'', 'use_sim_time':'true',
                'params_file':str(share/'config/nav2.yaml')}),
        Node(package='assign7_bringup', executable='semantic_regions_node.py',
             name='semantic_regions_node', parameters=[{'use_sim_time':True}], output='screen'),
    ])
