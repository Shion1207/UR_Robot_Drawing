"""Only the copied arena and TurtleBot simulator: no AMCL/Nav2/week-7 nodes."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,GroupAction,IncludeLaunchDescription,OpaqueFunction,SetEnvironmentVariable,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(path,args):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),launch_arguments=args.items())


def simulator(context,share,sim,ign):
    headless=LaunchConfiguration('headless').perform(context).lower()=='true'
    args=f'{share}/worlds/gesture_arena.sdf -r -v 3 -s'
    if headless:args+=' --headless-rendering'
    actions=[GroupAction(scoped=True,actions=[include(ign/'launch/ign_gazebo.launch.py',
        {'gz_args':args,'on_exit_shutdown':'true'})])]
    if not headless:
        actions.append(TimerAction(period=2.,actions=[GroupAction(scoped=True,actions=[
            include(ign/'launch/ign_gazebo.launch.py',{'gz_args':f'-g -v 2 --gui-config {sim}/gui/lite/gui.config',
                                                   'on_exit_shutdown':'false'})])]))
    return actions


def generate_launch_description():
    share=Path(get_package_share_directory('assign8_bringup'))
    sim=Path(get_package_share_directory('turtlebot4_ignition_bringup'))
    ign=Path(get_package_share_directory('ros_ign_gazebo'))
    create=Path(get_package_share_directory('irobot_create_ignition_bringup'))
    desc=Path(get_package_share_directory('turtlebot4_description'))
    cdesc=Path(get_package_share_directory('irobot_create_description'))
    plugins=Path(get_package_share_directory('irobot_create_ignition_plugins'))
    tbplugins=Path(get_package_share_directory('turtlebot4_ignition_gui_plugins'))
    return LaunchDescription([
        DeclareLaunchArgument('headless',default_value='false'),
        # The vendor diffdrive spawner is scheduled by an exit event after the
        # scoped spawn group pops. Keep its deferred namespace substitution valid.
        DeclareLaunchArgument('namespace',default_value='',choices=['']),
        SetEnvironmentVariable('QT_QPA_PLATFORM','xcb'),
        SetEnvironmentVariable('IGN_PARTITION','assign8'),
        # World argument is explicitly shared with ALL nested sensor bridges.
        DeclareLaunchArgument('world',default_value='gesture_arena',choices=['gesture_arena']),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',os.pathsep.join(map(str,[share/'worlds',sim/'worlds',
            create/'worlds',desc.parent,cdesc.parent]))),
        SetEnvironmentVariable('IGN_GUI_PLUGIN_PATH',os.pathsep.join(map(str,[tbplugins/'lib',plugins/'lib']))),
        OpaqueFunction(function=simulator,kwargs={'share':share,'sim':sim,'ign':ign}),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],output='screen'),
        GroupAction(scoped=True,actions=[include(sim/'launch/turtlebot4_spawn.launch.py',
            {'namespace':'','model':'lite','world':'gesture_arena','rviz':'false','slam':'false',
             'localization':'false','nav2':'false','use_sim_time':'true','x':'0.0','y':'0.0','z':'0.0','yaw':'0.0'})]),
    ])
