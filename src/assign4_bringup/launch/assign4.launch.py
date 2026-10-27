"""Run TurtleBot 4 simulation in one of the assignment's four modes."""

import os

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _is_mode(mode, expected):
    return IfCondition(PythonExpression(["'", mode, "' == '", expected, "'"]))


def _rviz_node(context):
    if LaunchConfiguration('rviz').perform(context).lower() != 'true':
        return []

    package_share = Path(get_package_share_directory('assign4_bringup'))
    mode = LaunchConfiguration('mode').perform(context)
    config_name = 'simulation.rviz' if mode == 'simulation' else 'navigation.rviz'

    return [Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(package_share / 'rviz' / config_name)],
        parameters=[{'use_sim_time': True}],
    )]


def _simulator_node(context, assignment_share, tb4_sim_share, ros_ign_share):
    world_name = LaunchConfiguration('world').perform(context)
    model_name = LaunchConfiguration('model').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    if world_name in ('small_room', 'gesture_arena'):
        world_file = assignment_share / 'worlds' / f'{world_name}.sdf'
    else:
        world_file = tb4_sim_share / 'worlds' / f'{world_name}.sdf'

    # Start the server separately so a slow / broken Qt GUI cannot prevent the
    # world from being created. This also removes the startup deadlock where
    # robot spawners wait forever at "Requesting list of world names".
    server_args = f'{world_file} -r -v 4 -s'
    if headless:
        server_args += ' --headless-rendering'

    server = GroupAction(
        scoped=True,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(ros_ign_share / 'launch' / 'ign_gazebo.launch.py')),
            launch_arguments={
                'gz_args': server_args,
                'on_exit_shutdown': 'true',
            }.items(),
        )],
    )
    actions = [server]

    if not headless:
        gui_args = (
            f'-g -v 4 --gui-config '
            f'{tb4_sim_share}/gui/{model_name}/gui.config'
        )
        gui = GroupAction(
            scoped=True,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(ros_ign_share / 'launch' / 'ign_gazebo.launch.py')),
                launch_arguments={
                    'gz_args': gui_args,
                    'on_exit_shutdown': 'false',
                }.items(),
            )],
        )
        actions.append(TimerAction(period=2.0, actions=[gui]))

    return actions


def generate_launch_description():
    tb4_sim_share = Path(get_package_share_directory('turtlebot4_ignition_bringup'))
    tb4_nav_share = Path(get_package_share_directory('turtlebot4_navigation'))
    assignment_share = Path(get_package_share_directory('assign4_bringup'))
    tb4_description_share = Path(get_package_share_directory('turtlebot4_description'))
    create_description_share = Path(get_package_share_directory('irobot_create_description'))
    create_sim_share = Path(get_package_share_directory('irobot_create_ignition_bringup'))
    create_plugins_share = Path(get_package_share_directory('irobot_create_ignition_plugins'))
    tb4_plugins_share = Path(get_package_share_directory('turtlebot4_ignition_gui_plugins'))
    ros_ign_share = Path(get_package_share_directory('ros_ign_gazebo'))

    mode = LaunchConfiguration('mode')
    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    namespace = LaunchConfiguration('namespace')
    map_file = LaunchConfiguration('map')

    resource_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        os.pathsep.join([
            str(assignment_share / 'worlds'),
            str(tb4_sim_share / 'worlds'),
            str(create_sim_share / 'worlds'),
            str(tb4_description_share.parent),
            str(create_description_share.parent),
        ]),
    )

    gui_plugin_path = SetEnvironmentVariable(
        'IGN_GUI_PLUGIN_PATH',
        os.pathsep.join([
            str(tb4_plugins_share / 'lib'),
            str(create_plugins_share / 'lib'),
        ]),
    )

    # The vendored Qt plugins do not contain the Wayland platform plugin.
    # Ubuntu's XWayland support is reliable for both Gazebo and RViz.
    qt_platform = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')

    simulator = OpaqueFunction(
        function=_simulator_node,
        kwargs={
            'assignment_share': assignment_share,
            'tb4_sim_share': tb4_sim_share,
            'ros_ign_share': ros_ign_share,
        },
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(tb4_sim_share / 'launch' / 'turtlebot4_spawn.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'model': model,
            'rviz': 'false',
            'slam': 'false',
            'localization': 'false',
            'nav2': 'false',
            'use_sim_time': 'true',
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(tb4_nav_share / 'launch' / 'slam.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': 'true',
            'sync': 'false',
            'params': str(assignment_share / 'config' / 'slam.yaml'),
        }.items(),
        condition=_is_mode(mode, 'mapping'),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(tb4_nav_share / 'launch' / 'localization.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': 'true',
            'params': str(assignment_share / 'config' / 'localization.yaml'),
            'map': map_file,
        }.items(),
        condition=_is_mode(mode, 'localization'),
    )

    localization_for_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(tb4_nav_share / 'launch' / 'localization.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': 'true',
            'params': str(assignment_share / 'config' / 'localization.yaml'),
            'map': map_file,
        }.items(),
        condition=_is_mode(mode, 'navigation'),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(tb4_nav_share / 'launch' / 'nav2.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': 'true',
            'params_file': str(assignment_share / 'config' / 'nav2.yaml'),
        }.items(),
        condition=_is_mode(mode, 'navigation'),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='simulation',
            choices=['simulation', 'mapping', 'localization', 'navigation'],
            description='Assignment mode to launch.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='small_room',
            choices=['small_room', 'gesture_arena', 'depot', 'maze', 'warehouse'],
            description='small_room for mapping; gesture_arena is open space for hand gesture demos.',
        ),
        DeclareLaunchArgument(
            'model',
            default_value='standard',
            choices=['standard', 'lite'],
            description='TurtleBot 4 model.',
        ),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            choices=['true', 'false'],
            description='Run only the Gazebo server with headless GPU rendering.',
        ),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'map',
            default_value=str(assignment_share / 'maps' / 'map.yaml'),
            description='Absolute path to the saved map YAML.',
        ),
        resource_path,
        gui_plugin_path,
        qt_platform,
        # Start RViz before turtlebot4_spawn.launch.py sets its own `rviz`
        # launch configuration to false.  Included launch descriptions share
        # the launch context, so evaluating this at the end would incorrectly
        # suppress the assignment's RViz window.
        OpaqueFunction(function=_rviz_node),
        simulator,
        clock_bridge,
        robot,
        slam,
        localization,
        localization_for_nav,
        nav2,
    ])

