# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    use_sim_time = True

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_base"), "config", "ekf.yaml"]
    )

    world_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_gazebo"), "worlds", "abu_sim_field.world"]
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'), 'launch', 'description.launch.py']
    )
    twist_mux_params = PathJoinSubstitution(
        [FindPackageShare("linorobot2_navigation"), 'launch', 'twist_mux.yaml']
    )

    game_controller_use = DeclareLaunchArgument(
        'use_real_game_pad',
        default_value='true',
        description='determine which game_pad to use'
    )
    game_pkg = Node(
        package='game_pad_pkg',
        executable='game_pad',
        name = 'game_pad',
        output = 'screen',
        condition = IfCondition(LaunchConfiguration('use_real_game_pad'))
    )
    game_sim = Node(
        package='game_pad_sim',
        executable='game_pad_sim',
        name = 'game_pad_sim',
        output = 'screen',
        condition = UnlessCondition(LaunchConfiguration('use_real_game_pad')) 
    )
    return LaunchDescription([
        # game_controller_use,
        # game_pkg,
        # game_sim,
        DeclareLaunchArgument(
            name='world', 
            default_value=world_path,
            description='Gazebo world'
        ),

        DeclareLaunchArgument(
            name='spawn_x', 
            default_value='-2.011176',
            description='Robot spawn position in X axis'
        ),

        DeclareLaunchArgument(
            name='spawn_y', 
            default_value='-6.699749',
            description='Robot spawn position in Y axis'
        ),

        DeclareLaunchArgument(
            name='spawn_z', 
            default_value='0.0',
            description='Robot spawn position in Z axis'
        ),
            
        DeclareLaunchArgument(
            name='spawn_yaw', 
            default_value='-1.5708',
            description='Robot spawn heading'
        ),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',  '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world')],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=[
                '-topic', 'robot_description', 
                '-entity', 'linorobot2', 
                '-x', LaunchConfiguration('spawn_x'),
                '-y', LaunchConfiguration('spawn_y'),
                '-z', LaunchConfiguration('spawn_z'),
                '-Y', LaunchConfiguration('spawn_yaw'),
            ]
        ),

        Node(
            package='linorobot2_gazebo',
            executable='command_timeout.py',
            name='command_timeout'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time}, 
                ekf_config_path
            ],
            remappings=[("odometry/filtered", "odom")]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'publish_joints': 'false',
            }.items()
        ),
        
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[twist_mux_params],
            remappings=[
                ('cmd_vel_out','cmd_vel_nav')
            ]
        ),

    ])

#sources: 
#https://navigation.ros.org/setup_guides/index.html#
#https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
#https://github.com/ros2/rclcpp/issues/940
