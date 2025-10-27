#include <moveit/move_group_interface/move_group_interface.h>          //da .hpp a .h
#include <moveit/planning_scene_interface/planning_scene_interface.h>  //da .hpp a .h

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc, char** argv)
{
    // Inizializzazione ROS2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("add_box");

    // Planning scene interface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // 1. Creazione del pavimento
    moveit_msgs::msg::CollisionObject floor;
    floor.header.frame_id = "world"; // Il riferimento è il frame "world"
    floor.id = "floor"; // Nome univoco

    shape_msgs::msg::SolidPrimitive floor_primitive;
    floor_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    floor_primitive.dimensions = {3.0, 3.0, 0.1}; // Dimensioni del pavimento

    geometry_msgs::msg::Pose floor_pose;
    floor_pose.position.x = 0.0;
    floor_pose.position.y = 0.0;
    
    // Calcolare la posizione z in modo che la superficie superiore del pavimento tocchi il piano di riferimento
    floor_pose.position.z = -(floor_primitive.dimensions[2] / 2.0); // metà dell'altezza

    floor_pose.orientation.w = 1.0;

    floor.primitives.push_back(floor_primitive);
    floor.primitive_poses.push_back(floor_pose);
    floor.operation = floor.ADD;

    // 2. Creazione della seconda box
    moveit_msgs::msg::CollisionObject box;
    box.header.frame_id = "world";
    box.id = "box1";

    shape_msgs::msg::SolidPrimitive box_primitive;
    box_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    box_primitive.dimensions = {0.25, 0.25, 0.5}; // Dimensioni della seconda box

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.5; // Posizione relativa al frame "world"
    box_pose.position.y = 0.25;
    
    // Calcolare la posizione z in modo che la superficie inferiore del pavimento tocchi il piano di riferimento
    box_pose.position.z = (box_primitive.dimensions[2] / 2.0); // metà dell'altezza
    // OSS: Un problema potrebbe essere aver impostato la coincidenza della superficie superiore di una box con la superfice inferiore dell'altra 
    //      Quindi in quel caso si potrebbe pensare di aggiungere un piccolo offset (poi vediamo)

    box_pose.orientation.w = 1.0;

    box.primitives.push_back(box_primitive);
    box.primitive_poses.push_back(box_pose);
    box.operation = box.ADD;

    // Aggiunta degli oggetti alla scena
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(floor);
    collision_objects.push_back(box);

    // Aggiungi gli oggetti alla scena di collisione
    RCLCPP_INFO(node->get_logger(), "Aggiungo gli oggetti alla scena...");
    planning_scene_interface.addCollisionObjects(collision_objects);

    // Fermo il nodo per visualizzare in RViz
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
