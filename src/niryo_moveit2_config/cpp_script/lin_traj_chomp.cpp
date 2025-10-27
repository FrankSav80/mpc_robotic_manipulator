#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include <Eigen/Geometry> // Per Eigen::Isometry3d
#include <tf2_eigen/tf2_eigen.hpp> // Per tf2::fromMsg

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <fstream>

int main(int argc, char * argv[])
{
// Start up ROS 2
rclcpp::init(argc, argv);

// Creates a node named "lin_traj_chomp". The node is set up to automatically
// handle any settings (parameters) we might want to change later without editing the code.
auto const node = std::make_shared<rclcpp::Node>(
    "lin_traj_chomp",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
);

// Creates a "logger" that we can use to print out information or error messages
// as our program runs.
auto const logger = rclcpp::get_logger("lin_traj_chomp");

// This code creates a separate thread, which is like a mini-program running alongside our main program.
// This thread uses a ROS 2 "executor" to continuously process information about the robot's state.
// By running this in its own thread, our ROS 2 node can keep getting updates about the robot without
// interrupting the main flow of our program.
rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node);
auto spinner = std::thread([&executor]() { executor.spin(); });

// Create the MoveIt MoveGroup Interfaces
// These interfaces are used to plan and execute movements, set target poses,
// and perform other motion-related tasks for each respective part of the robot.
// The use of auto allows the compiler to automatically deduce the type of variable.
using moveit::planning_interface::MoveGroupInterface;
auto arm_group_interface = MoveGroupInterface(node, "niryo_arm");

// Ottieniamo la posa corrente dell'end-effector prima della pianificazione
auto current_pose = arm_group_interface.getCurrentPose();

RCLCPP_INFO(logger, "Posa iniziale dell'end-effector:");
RCLCPP_INFO(logger, "  Posizione: x=%.3f, y=%.3f, z=%.3f",
            current_pose.pose.position.x,
            current_pose.pose.position.y,
            current_pose.pose.position.z);
RCLCPP_INFO(logger, "  Orientamento: qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w);

// Specify a planning pipeline to be used for further planning
arm_group_interface.setPlanningPipelineId("chomp");

// Specify the maximum amount of time in seconds to use when planning
arm_group_interface.setPlanningTime(1.0);

// Set a scaling factor for optionally reducing the maximum joint velocity. Allowed values are in (0,1].
arm_group_interface.setMaxVelocityScalingFactor(1.0);

//  Set a scaling factor for optionally reducing the maximum joint acceleration. Allowed values are in (0,1].
arm_group_interface.setMaxAccelerationScalingFactor(1.0);

// Display helpful logging messages on the terminal
RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());
RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());

// Construct and initialize MoveItVisualTools
auto moveit_visual_tools =
    moveit_visual_tools::MoveItVisualTools{ node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                            arm_group_interface.getRobotModel() };

// Erase all existing visual markers from the RViz visualization.
moveit_visual_tools.deleteAllMarkers();

// Load the remote control interface for MoveIt visual tools.
moveit_visual_tools.loadRemoteControl();

// Lambda function to draw a title in the RViz visualization
auto const draw_title = [&moveit_visual_tools](auto text) {
    // Nested lambda to create a pose for the text
    auto const text_pose = [] {
        // Create an identity transform
        auto msg = Eigen::Isometry3d::Identity();
        // Set the z-coordinate to 1.0 (1 meter above the origin)
        msg.translation().z() = 1.0;
        return msg;
    }();

    // Publish the text to RViz
    // Parameters: pose, text content, color (WHITE), and size (XLARGE)
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
};

// Lambda function to display a prompt message in RViz
auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
};

// Lambda function to visualize a trajectory as a line in RViz
auto const draw_trajectory_tool_path =
    [&moveit_visual_tools, jmg = arm_group_interface.getRobotModel()->getJointModelGroup("niryo_arm")](
    auto const trajectory) {moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);};

// Set a target pose for the end effector of the arm
auto const arm_target_pose1 = [&node]{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.header.stamp = node->now();
    msg.pose.position.x = 0.111;
    msg.pose.position.y = 0.272;
    msg.pose.position.z = 0.268;
    msg.pose.orientation.x = -0.351;
    msg.pose.orientation.y = 0.150;
    msg.pose.orientation.z = 0.850;
    msg.pose.orientation.w = 0.364;
    return msg;
}();

auto const arm_target_pose2 = [&node]{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.header.stamp = node->now();
    msg.pose.position.x = 0.120;
    msg.pose.position.y = -0.268;
    msg.pose.position.z = 0.268;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.382;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 0.923;
    return msg;
}();

std::ofstream ee_file("/home/francesco/lab_ROS2/ws_moveit2/src/niryo_moveit2_config/cpp_script/ref_ee_traj_prova.txt");   // CAMBIATO NOME FILE PER 100ms

//  Funzione
auto plan_and_execute = [&](const geometry_msgs::msg::PoseStamped& arm_target_pose) -> bool {
    // Trova il modello cinematico del robot
    const moveit::core::JointModelGroup* joint_model_group =
    arm_group_interface.getCurrentState()->getJointModelGroup("niryo_arm");
    // Crea uno stato robotico per calcolare l'IK
    moveit::core::RobotState target_state(*arm_group_interface.getCurrentState());
    // Calcola la cinematica inversa per trovare i valori articolari corrispondenti alla arm_target_pose
    bool found_ik = target_state.setFromIK(joint_model_group, arm_target_pose.pose, true);

    if (found_ik) {
    std::vector<double> joint_target;
    target_state.copyJointGroupPositions(joint_model_group, joint_target);
    // Stampa i valori articolari trovati dall'IK
    RCLCPP_INFO(node->get_logger(), "IK trovata! Target articolare:");
    for (size_t i = 0; i < joint_target.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "  Joint %zu: %f", i, joint_target[i]);
    }
    // Imposta il target articolare ottenuto dall'IK
    arm_group_interface.setJointValueTarget(joint_target);
    } else {
    RCLCPP_ERROR(node->get_logger(), "IK non trovata per la posa desiderata!");
    RCLCPP_INFO(node->get_logger(), "Posizione desiderata: x=%f, y=%f, z=%f",
                arm_target_pose.pose.position.x, arm_target_pose.pose.position.y, arm_target_pose.pose.position.z);
    return false;
    }

    // Create a plan to that target pose
    // This will give us two things:
    // 1. Whether the planning was successful (stored in 'success')
    // 2. The actual motion plan (stored in 'plan')
    auto const [success, plan] = [&arm_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(arm_group_interface.plan(msg));
    return std::make_pair(ok, msg);
    }();

    // Try to execute the movement plan if it was created successfully
    // If the plan wasn't successful, report an error
    // Execute the plan
    if (success)
    {

    // Costruisci l'oggetto RobotTrajectory a partire dal messaggio
    robot_trajectory::RobotTrajectory robot_traj(
        arm_group_interface.getRobotModel(),
        "niryo_arm"
    );
    robot_traj.setRobotTrajectoryMsg(
        *arm_group_interface.getCurrentState(),
        plan.trajectory_
    );

    std::vector<std::vector<double>> sampled_joint_trajectory;
    std::vector<std::vector<double>> sampled_ee_trajectory;

    double dt_sampling = 0.1;  // 100 ms  ---------- CAMBIATO, PRIMA PROVA FATTA CON 40ms---------
    double traj_duration = robot_traj.getDuration();

    for (double t = 0.0; t <= traj_duration; t += dt_sampling) {
        // Crea uno shared_ptr a RobotState
        moveit::core::RobotStatePtr state = std::make_shared<moveit::core::RobotState>(robot_traj.getFirstWayPoint());
        robot_traj.getStateAtDurationFromStart(t, state);

        // Estrai posizioni articolari
        std::vector<double> joint_positions;
        state->copyJointGroupPositions("niryo_arm", joint_positions);
        sampled_joint_trajectory.push_back(joint_positions);

        // Estrai pose dell'end-effector
        const moveit::core::LinkModel* ee_link = state->getLinkModel("tool_link");  // Assicurati che sia il nome giusto
        const Eigen::Isometry3d& ee_pose = state->getGlobalLinkTransform(ee_link);
        const Eigen::Vector3d& position = ee_pose.translation();
        const Eigen::Quaterniond orientation(ee_pose.rotation());

        std::vector<double> ee_pose_vec = {
            position.x(), position.y(), position.z(),
            orientation.x(), orientation.y(), orientation.z(), orientation.w()
        };
        sampled_ee_trajectory.push_back(ee_pose_vec);
    }

    // Log posizioni articolari
    for (size_t i = 0; i < sampled_joint_trajectory.size(); ++i) {
        std::ostringstream joint_line;
        joint_line << "Joint[" << i << "]: ";
        for (double val : sampled_joint_trajectory[i])
            joint_line << val << " ";
        RCLCPP_INFO(logger, "%s", joint_line.str().c_str());
    }

    // Log pose EE
    for (size_t i = 0; i < sampled_ee_trajectory.size(); ++i) {
        std::ostringstream ee_line;
        for (double val : sampled_ee_trajectory[i]) {
            ee_line << val << " ";
        }
        ee_file << ee_line.str() << "\n";  // Scrivi su file
    }
    ee_file.close();

    RCLCPP_INFO(logger, "Traiettoria campionata in %lu punti da CHOMP.", sampled_joint_trajectory.size());


    // Visualize the planned trajectory in RViz
    draw_trajectory_tool_path(plan.trajectory_);
    // Trigger the visualization update
    moveit_visual_tools.trigger();

    // Prompt the user to continue to execution
    // prompt("Press 'next' in the RvizVisualToolsGui window to execute");

    // Update the title in RViz to show we're executing the plan
    // draw_title("Executing");

    // Trigger another visualization update
    moveit_visual_tools.trigger();
    // Execute the planned motion
    arm_group_interface.execute(plan);

    // Aspetta un attimo per aggiornare lo stato del robot
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Ottieni la posa finale dell'end-effector dopo l'esecuzione
    auto final_pose = arm_group_interface.getCurrentPose();

    RCLCPP_INFO(logger, "Posa finale dell'end-effector:");
    RCLCPP_INFO(logger, "  Posizione: x=%.3f, y=%.3f, z=%.3f",
                final_pose.pose.position.x,
                final_pose.pose.position.y,
                final_pose.pose.position.z);
    RCLCPP_INFO(logger, "  Orientamento: qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
                final_pose.pose.orientation.x,
                final_pose.pose.orientation.y,
                final_pose.pose.orientation.z,
                final_pose.pose.orientation.w);
    }
    else
    {
    // If planning failed, update the title in RViz
    draw_title("Planning Failed!");

    // Trigger the visualization update
    moveit_visual_tools.trigger();

    // Log an error message
    RCLCPP_ERROR(logger, "Planning failed!");
    }
    // Exit
    return true;
};

// Creazione del pavimento
auto const create_floor = [frame_id = arm_group_interface.getPlanningFrame(), &node, &logger] {
    moveit_msgs::msg::CollisionObject floor;
    floor.header.frame_id = frame_id;
    floor.header.stamp = node->now();
    floor.id = "floor";

    shape_msgs::msg::SolidPrimitive floor_primitive;
    floor_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    floor_primitive.dimensions = {3.0, 3.0, 0.1};

    geometry_msgs::msg::Pose floor_pose;
    floor_pose.position.x = 0.0;
    floor_pose.position.y = 0.0;
    floor_pose.position.z = -(floor_primitive.dimensions[2] / 2.0); // Faccio si che la superficie superiore del pavimento tocchi la base del robot
    floor_pose.orientation.w = 1.0;

    floor.primitives.push_back(floor_primitive);
    floor.primitive_poses.push_back(floor_pose);
    floor.operation = floor.ADD;

    RCLCPP_INFO(logger, "Pavimento aggiunto alla scena di collisione.");
    return floor;
}();

// Applica il pavimento alla scena
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
planning_scene_interface.applyCollisionObjects({create_floor});
moveit_visual_tools.trigger();
rclcpp::sleep_for(std::chrono::seconds(1)); // Tempo per aggiornare la scena

// Esegui il primo target
if (!plan_and_execute(arm_target_pose1)) {
    RCLCPP_ERROR(logger, "Errore nel raggiungere la prima posa, interrompo il programma.");
    return 1;
}

// Wait for the spinner thread to finish
spinner.join();
// Exit the program
return 0;
}
