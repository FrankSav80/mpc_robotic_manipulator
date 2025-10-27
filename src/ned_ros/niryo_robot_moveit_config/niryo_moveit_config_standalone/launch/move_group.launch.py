import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('hardware_version', default_value='ned', description='Hardware version of the robot'),
        DeclareLaunchArgument('load_robot_description', default_value='false', description='Load robot description'),
        DeclareLaunchArgument('simulation_mode', default_value='false', description='Run in simulation mode'),
        DeclareLaunchArgument('debug', default_value='false', description='Enable debug mode'),
        DeclareLaunchArgument('info', default_value='false', description='Verbose mode'),
        DeclareLaunchArgument('allow_trajectory_execution', default_value='true', description='Allow trajectory execution'),

        # Log debug and info messages
        LogInfo(msg="Debug mode is: {}".format(LaunchConfiguration('debug'))),
        LogInfo(msg="Verbose mode is: {}".format(LaunchConfiguration('info'))),

        # Load URDF, SRDF, and configuration files
        Node(
            package='niryo_moveit_config_standalone',
            executable='planning_context.launch',
            name='planning_context',
            output='screen',
            parameters=[{'hardware_version': LaunchConfiguration('hardware_version'),
                         'simulation_mode': LaunchConfiguration('simulation_mode'),
                         'load_robot_description': LaunchConfiguration('load_robot_description')}],
        ),

        # Trajectory execution and sensor manager, based on conditions
        Node(
            package='niryo_moveit_config_standalone',
            executable='trajectory_execution.launch',
            name='trajectory_execution',
            output='screen',
            condition=IfCondition(LaunchConfiguration('allow_trajectory_execution')),
            parameters=[{'hardware_version': LaunchConfiguration('hardware_version')}]
        ),

        Node(
            package='niryo_moveit_config_standalone',
            executable='sensor_manager.launch',
            name='sensor_manager',
            output='screen',
            condition=IfCondition(LaunchConfiguration('allow_trajectory_execution')),
        ),
    ])

