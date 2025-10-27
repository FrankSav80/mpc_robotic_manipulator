import os
from launch import LaunchDescription
from launch_ros.descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():

    # ARGUMENTS ---------------------------------------------------------------

    rviz_arg = DeclareLaunchArgument("rviz_gui", default_value="false",
                                                 description="Flag to enable rviz")

    # current package path
    pkg_share_path = get_package_share_directory("niryo_robot_description")               # CAMBIATO (nome cartella)

    # Rviz config path
    rviz_config_path = PathJoinSubstitution(
        [pkg_share_path, "config", "default_config.rviz"]                                 # CAMBIATO (file)
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),                         
            " ",
            PathJoinSubstitution(
                [pkg_share_path, "urdf/ned2", "niryo_ned2.urdf.xacro"]                    # CAMBIATO (file)
            ),
            " ",
            "name:=niryo_ned2"                                                            # CAMBIATO (optional-nome)
        ]
    )

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=None)}

    # NODES -----------------------------------------------------------------

    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(LaunchConfiguration("rviz_gui"))
    )

    # GAZEBO -----------------------------------------------------------------

    description_share = os.path.join(get_package_prefix("niryo_robot_description"), "share")                # CAMBIATO (nome cartella)
    gazebo_env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", description_share)

    gazebo_server = ExecuteProcess(
        cmd=[
              "gzserver",
              "--verbose",
              "-s", "libgazebo_ros_factory.so",
              os.path.join(pkg_share_path, "worlds", "empty.world"),             # NUOVA CARTELLA E FILE 
            ],
        output="screen"
    )
    
    gazebo_client = ExecuteProcess(cmd=["gzclient"], output="screen")

    spawn_robot = Node(
        name="spawn_robot",
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-entity", "niryo_ned2",                       # CAMBIATO (optional-nome)
                   "-topic", "/robot_description",
                   "-x", "0.0",
                   "-y", "0.0",
                   "-z", "0.5"]
    )

    # ROS 2 CONTROL ---------------------------------------------------------


    # Return Launch Function -------------------------------------------------

    return LaunchDescription(
        [
            rviz_arg,
            rviz_node,
            robot_state_publisher_node,
            
            gazebo_env_var,
            gazebo_server,
            gazebo_client,
            spawn_robot
        ]
    )
