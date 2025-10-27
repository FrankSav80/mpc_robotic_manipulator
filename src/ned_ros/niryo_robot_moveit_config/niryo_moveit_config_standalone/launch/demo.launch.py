from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    hardware_version = DeclareLaunchArgument("hardware_version", default_value="ned")
    simulation_mode = DeclareLaunchArgument("simulation_mode", default_value="false")
    db = DeclareLaunchArgument("db", default_value="false")
    db_path = DeclareLaunchArgument(
        "db_path",
        default_value=os.path.join(
            get_package_share_directory("niryo_moveit_config_standalone"),
            "default_warehouse_mongo_db"
        ),
    )
    debug = DeclareLaunchArgument("debug", default_value="false")

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_gui": False}],
        remappings=[("/source_list", "/move_group/fake_controller_joint_states")],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        respawn=True,
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("niryo_moveit_config_standalone"),
                "launch",
                "move_group.launch.py"
            )
        ),
        launch_arguments={
            "load_robot_description": "true",
            "hardware_version": LaunchConfiguration("hardware_version"),
            "simulation_mode": LaunchConfiguration("simulation_mode"),
            "debug": LaunchConfiguration("debug"),
            "info": "true",
            "allow_trajectory_execution": "true",
        }.items(),
    )

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("niryo_moveit_config_standalone"),
                "launch",
                "moveit_rviz.launch.py"
            )
        ),
        launch_arguments={
            "config": "true",
            "debug": LaunchConfiguration("debug"),
        }.items(),
    )

    warehouse_db_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("niryo_moveit_config_standalone"),
                "launch",
                "default_warehouse_db.launch.py"
            )
        ),
        launch_arguments={
            "moveit_warehouse_database_path": LaunchConfiguration("db_path"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("db"))
    )

    return LaunchDescription([
        hardware_version,
        simulation_mode,
        db,
        db_path,
        debug,
        joint_state_publisher,
        robot_state_publisher,
        move_group_launch,
        moveit_rviz_launch,
        warehouse_db_launch,
    ])

