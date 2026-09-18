import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory('chapt10_bringup')
    fishbot_dir = get_package_share_directory('fishbot_description')
    nav2_dir = get_package_share_directory('fishbot_navigation2')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    planner_dir = get_package_share_directory('chapt10_planner')
    mpc_dir = get_package_share_directory('chapt10_control')

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    map_yaml = LaunchConfiguration('map')
    planner_parameters_file = LaunchConfiguration('planner_params')
    mpc_parameters_file = LaunchConfiguration('mpc_params')

    gui_cmd = DeclareLaunchArgument(
        'gui', default_value='true', description='Set to "false" to run Gazebo headless')
    rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true', description='Set to "false" to skip RViz2')
    map_cmd = DeclareLaunchArgument(
        'map', default_value=os.path.join(nav2_dir, 'maps', 'room.yaml'),
        description='Occupancy grid map (chapt7 fishbot_navigation2/maps/room.yaml)')
    planner_params_cmd = DeclareLaunchArgument(
        'planner_params', default_value=os.path.join(planner_dir, 'config', 'planner.yaml'),
        description='Path planner / ROS2 configuration file')
    mpc_params_cmd = DeclareLaunchArgument(
        'mpc_params', default_value=os.path.join(mpc_dir, 'config', 'mpc.yaml'),
        description='MPC / ROS2 configuration file')


    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={'gui': gui,
                          'world': os.path.join(fishbot_dir, 'world', 'custom_room.world')}.items()
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', os.path.join(fishbot_dir, 'urdf', 'fishbot',
                                                'fishbot.urdf.xacro')]),
                value_type=str),
            'use_sim_time': True,
        }]
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        emulate_tty=True,
        arguments=['-topic', '/robot_description', '-entity', 'fishbot']
    )

    controller_spawner = TimerAction(period=8.0, actions=[
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['fishbot_joint_state_broadcaster', '--controller-manager-timeout', '60']),
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['fishbot_diff_drive_controller', '--controller-manager-timeout', '60']),
    ])

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        emulate_tty=True,
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': True}]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(bringup_dir, 'config', 'amcl.yaml'), {'use_sim_time': True}]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        emulate_tty=True,
        parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl'],
                     'use_sim_time': True}]
    )

    path_planner_node = Node(
        package='chapt10_planner',
        executable='planner_node',
        name='planner_node',
        output='screen',
        emulate_tty=True,
        parameters=[ParameterFile(planner_parameters_file, allow_substs=True)]
    )

    mpc_controller_node = Node(
        package='chapt10_control',
        executable='mpc_node',
        name='mpc_node',
        output='screen',
        emulate_tty=True,
        parameters=[ParameterFile(mpc_parameters_file, allow_substs=True)]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz),
        arguments=['-d', os.path.join(bringup_dir, 'rviz', 'navigation.rviz')]
    )

    ld = LaunchDescription()

    ld.add_action(gui_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(map_cmd)
    ld.add_action(planner_params_cmd)
    ld.add_action(mpc_params_cmd)

    ld.add_action(gazebo_launch)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_node)
    ld.add_action(controller_spawner)
    ld.add_action(map_server_node)
    ld.add_action(amcl_node)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(path_planner_node)
    ld.add_action(mpc_controller_node)
    ld.add_action(rviz2_node)

    return ld
