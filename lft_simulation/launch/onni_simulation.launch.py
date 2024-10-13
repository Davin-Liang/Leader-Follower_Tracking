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
from launch.conditions import IfCondition, UnlessCondition
from launch.actions.append_environment_variable import AppendEnvironmentVariable
# from launch.substitutions import EqualsSubstitution

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('lft_simulation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Specify xacro path
    urdf_dir_3d = get_package_share_path('lft_simulation') / 'urdf' / 'simulation_3d_waking_robot.xacro'
    urdf_dir_2d = get_package_share_path('lft_simulation') / 'urdf' / 'simulation_2d_waking_robot.xacro'

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz', default='false')
    use_3d_lidar = LaunchConfiguration('lidar_3d')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_3d_lidar_cmd = DeclareLaunchArgument(
        'lidar_3d',
        default_value='True',
    )

    # Specify the actions
    gazebo_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    )

    robot_joint_state_publisher_2d = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', str(urdf_dir_2d)]), value_type=str
            ),
        }],
        output='screen',
        condition=UnlessCondition(use_3d_lidar)
    )

    robot_state_publisher_2d = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', str(urdf_dir_2d)]), value_type=str
            ),
        }],
        output='screen',
        condition=UnlessCondition(use_3d_lidar)
    )

    robot_joint_state_publisher_3d = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', str(urdf_dir_3d)]), value_type=str
            ),
        }],
        output='screen',
        condition=IfCondition(use_3d_lidar)
    )

    robot_state_publisher_3d = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', str(urdf_dir_3d)]), value_type=str
            ),
        }],
        output='screen',
        condition=IfCondition(use_3d_lidar)
    )

    def create_robot_launch_group():
        return GroupAction(
            actions=[
                Node(
                    package='joint_state_publisher',
                    executable='joint_state_publisher',
                    name='joint_state_publisher',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'robot_description': ParameterValue(
                            Command(['xacro ', str(urdf_dir_2d)]), value_type=str
                        ),
                    }],
                    output='screen',
                    condition=UnlessCondition(use_3d_lidar)  # 2D lidar mode
                ),
                Node(
                    package='joint_state_publisher',
                    executable='joint_state_publisher',
                    name='joint_state_publisher',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'robot_description': ParameterValue(
                            Command(['xacro ', str(urdf_dir_3d)]), value_type=str
                        ),
                    }],
                    output='screen',
                    condition=IfCondition(use_3d_lidar)  # 3D lidar mode
                ),
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'robot_description': ParameterValue(
                            Command(['xacro ', str(urdf_dir_2d)]), value_type=str
                        ),
                    }],
                    output='screen',
                    condition=UnlessCondition(use_3d_lidar) # 2D lidar mode
                ),
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'robot_description': ParameterValue(
                            Command(['xacro ', str(urdf_dir_3d)]), value_type=str
                        ),
                    }],
                    output='screen',
                    condition=IfCondition(use_3d_lidar)  # 3D lidar mode
                ),
            ]
        )

    def create_gazebo_launch_group():
        # 设置 robot1 和 robot2 的初始位置和方向
        robot_config = {
            'x': '0.0',
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
                    # name='leader_spawn_entity',
                    arguments=[
                        '-entity', 'robot',
                        '-topic', 'robot_description', 
                        '-x', robot_config['x'],
                        '-y', robot_config['y'],
                        '-z', robot_config['z'],
                        '-Y', robot_config['yaw']
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
    robot_launch_group = create_robot_launch_group()

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_3d_lidar_cmd)
    ld.add_action(gazebo_client_launch)
    ld.add_action(robot_joint_state_publisher_2d)
    ld.add_action(robot_state_publisher_2d)
    ld.add_action(robot_joint_state_publisher_3d)
    ld.add_action(robot_state_publisher_3d)
    # ld.add_action(robot_launch_group)

    ld.add_action(bringup_world_cmd_group) # type: ignore

    return ld
