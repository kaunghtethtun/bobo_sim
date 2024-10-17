#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnProcessStart
import xacro

def generate_launch_description():
    description_pkg = get_package_share_directory('romrobots_description')
    urdf_file = os.path.join(description_pkg,'urdf', 'yoyo.urdf')

    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        # arguments=['-database', 'yoyo_tall_ros', '-entity', 'yoyo_tall_ros',
        arguments=['-file', urdf_file, '-entity', 'yoyo_standalone',
                   "-x", '0.0',
                   "-y", '0.0',
                   "-z", '0.1'],
        output='screen'
    )
    

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller"],
        parameters=[{
            'diff_cont.enable_odom_tf': LaunchConfiguration('enable_odom_tf')
        }]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcaster"],
    )

    twist_mux_params = os.path.join(get_package_share_directory('romrobots_gazebo'), 'config', 'twist_mux.yaml')
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_controller/cmd_vel_unstamped')]
    )
    # Delay start of joint_state_broadcaster_spawner after `spawn_robot_node`
    delay_joint_state_broadcaster_spawner_after_spawn_robot_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster_spawner`
    delay_diff_drive_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )


    return LaunchDescription(
        [
            DeclareLaunchArgument('enable_odom_tf', default_value='true', description='Enable or disable the publishing of the odom to base_link tf'),
            spawn_robot_node,
            delay_joint_state_broadcaster_spawner_after_spawn_robot_node,
            delay_diff_drive_spawner_after_joint_state_broadcaster_spawner,
            twist_mux_node,
        ]
    )
