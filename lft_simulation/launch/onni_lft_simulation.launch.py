#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory, get_package_share_path

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import IfCondition
from launch.actions.append_environment_variable import AppendEnvironmentVariable


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('lft_simulation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Specify xacro path
    leader_urdf_dir = get_package_share_path('lft_simulation') / 'urdf' / 'simulation_leader_waking_robot.xacro'
    slave_urdf_dir = get_package_share_path('lft_simulation') / 'urdf' / 'simulation_slave_waking_robot.xacro'

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz', default='false')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    # Specify the actions
    gazebo_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    )

    leader_robot_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='leader_joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', str(leader_urdf_dir)]), value_type=str
            ),
        }],
        remappings=[('/robot_description', '/robot1_description')],
        output='screen'
    )

    leader_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='leader_robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', str(leader_urdf_dir)]), value_type=str
            ),
        }],
        remappings=[('/robot_description', '/robot1_description')],
        output='screen'
    )

    leader_tf2 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_leader',
            arguments=['0', '0', '0', '0', '0', '0', '/map', '/leader/odom']
        )

    slave_robot_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='slave_joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', str(slave_urdf_dir)]), value_type=str
            ),
        }],
        remappings=[('/robot_description', '/robot2_description')],
        output='screen'
    )

    slave_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='slave_robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', str(slave_urdf_dir)]), value_type=str
            ),
        }],
        remappings=[('/robot_description', '/robot2_description')],
        output='screen'
    )

    slave_tf2 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_slave',
            arguments=['0', '0', '0', '0', '0', '0', '/map', '/slave/odom']
        )

    def create_gazebo_launch_group():
        # 设置 robot1 和 robot2 的初始位置和方向
        robot1_config = {
            'x': '0.0',
            'y': '0.0',
            'z': '0.0',
            'yaw': '0.0',
        }

        robot2_config = {
            'x': '2.0',  # 例如，设置 robot2 在 x 方向偏移 2.0 米
            'y': '0.0',
            'z': '0.0',
            'yaw': '0.0',
        }

        return GroupAction(
            actions=[
                # 启动 robot1
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='leader_spawn_entity',
                    arguments=[
                        '-entity', 'leader_robot',
                        '-topic', 'robot1_description', 
                        '-x', robot1_config['x'],
                        '-y', robot1_config['y'],
                        '-z', robot1_config['z'],
                        '-Y', robot1_config['yaw']
                    ],
                ),
                # 启动 robot2
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='slave_spawn_entity',
                    arguments=[
                        '-entity', 'slave_robot',
                        '-topic', 'robot2_description',
                        '-x', robot2_config['x'],
                        '-y', robot2_config['y'],
                        '-z', robot2_config['z'],
                        '-Y', robot2_config['yaw']
                    ],
                ),
                # 启动 Gazebo 并加载空白世界
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
                    launch_arguments={'world': 'empty.world'}.items(),
                )
            ]
        )


    bringup_world_cmd_group = create_gazebo_launch_group()

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(gazebo_client_launch)
    ld.add_action(leader_robot_joint_state_publisher)
    ld.add_action(leader_robot_state_publisher)
    ld.add_action(leader_tf2)
    ld.add_action(slave_robot_joint_state_publisher)
    ld.add_action(slave_robot_state_publisher)
    ld.add_action(slave_tf2)

    ld.add_action(bringup_world_cmd_group) # type: ignore

    return ld
