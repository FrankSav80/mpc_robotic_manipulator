import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction # AGGIUNTO TimerAction - per nodo mpc_controller_node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define xacro mappings for the robot description file (adjust if needed)
    launch_arguments = {
        "use_fake_hardware": "true",  # Mock hardware for simulation
        "dof": "6",  # Degrees of freedom for Niryo Ned2
    }

    # Load the robot configuration using MoveItConfigsBuilder
    moveit_config = (
        MoveItConfigsBuilder("niryo_ned2", package_name="niryo_moveit2_config")
        .robot_description(mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
             pipelines=["ompl", "chomp"],
             default_planning_pipeline="chomp"   # Aggiunto per settare come pipeline di default chomp
        )
        .to_moveit_configs()
    )

    # Move Group Node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            # {"use_sim_time": True},
            ],
    )

    # RViz configuration
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("niryo_moveit2_config"), "config", rviz_base]
    )

    # RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF Node
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            # {"use_sim_time": True},
            ],
    )

    # ros2_control setup for mock hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("niryo_moveit2_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ros2_controllers_path,
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--ros-args",                  # Aggiunto
            "--remap", "/frame_id:=world"  # Aggiunto
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["niryo_arm_controller", "-c", "/controller_manager"],
    )

#    forward_velocity_controller_spawner = Node(
#        package="controller_manager",
#        executable="spawner",
#        arguments=["forward_velocity_controller", "-c", "/controller_manager"],
#    )

    velocity_controller_spawner= Node(
        package="controller_manager",
        executable="spawner",
        arguments=["niryo_velocity_controller", "-c", "/controller_manager"],
    )

    # AGGIUNGO
    # Nodo dynamic_scene_setup
    dynamic_scene_setup_node = Node(
        package="niryo_moveit2_config",
        executable="dynamic_scene_setup",
        output="screen",
        parameters=[
            moveit_config.robot_description,  # Carica URDF
            moveit_config.robot_description_semantic,  # Carica SRDF
            moveit_config.robot_description_kinematics,  # Carica kinematics.yaml
            moveit_config.planning_pipelines,  # Carica la configurazione delle pipeline di pianificazione
        ],
    )

    # Avvia il nodo dynamic_scene_setup dopo un ritardo di 20 secondi
    delayed_dynamic_scene_setup_node = TimerAction(
        period=22.0,  # Ritardo di 20 secondi
        actions=[dynamic_scene_setup_node],
    )

    # Nodo MPC Controller
    mpc_controller_node = Node(
        package="niryo_moveit2_config",
        executable="mpc_controller",
        output="screen",
        parameters=[],
    )

    # Avvia il nodo mpc_controller dopo un ritardo di 30 secondi
    delayed_mpc_controller_node = TimerAction(
        period=30.0,  # Ritardo di 30 secondi
        actions=[mpc_controller_node],
    )

    return LaunchDescription(
        [
            rviz_config_arg,
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            # arm_controller_spawner,
            # forward_velocity_controller_spawner,
            velocity_controller_spawner,
            delayed_dynamic_scene_setup_node,
            delayed_mpc_controller_node,
        ]
    )
