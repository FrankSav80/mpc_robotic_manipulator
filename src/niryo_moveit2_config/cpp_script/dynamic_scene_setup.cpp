#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char * argv[])
{
// Start up ROS 2
rclcpp::init(argc, argv);

auto const node = std::make_shared<rclcpp::Node>(
    "dynamic_scene_setup",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
);

auto const logger = rclcpp::get_logger("dynamic_scene_setup");

rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node);
auto spinner = std::thread([&executor]() { executor.spin(); });

// Publisher per la posa dell'ostacolo
// Questo topic pubblica la posizione attuale della box (ostacolo dinamico) per essere usata dall'MPC
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr obstacle_pose_publisher;
obstacle_pose_publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>("/obstacle_pose", 10);

// Creazione del pavimento
auto const create_floor = [frame_id = "world", &node, &logger] {
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
    floor_pose.position.z = -(floor_primitive.dimensions[2] / 2.0);
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
rclcpp::sleep_for(std::chrono::seconds(1));

// Creazione della box
auto const create_box = [frame_id = "world", &node, &logger] {
    moveit_msgs::msg::CollisionObject box;
    box.header.frame_id = frame_id;
    box.header.stamp = node->now();
    box.id = "box1";

    shape_msgs::msg::SolidPrimitive box_primitive;
    box_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    box_primitive.dimensions = {0.10, 0.10, 0.10}; // cubo 10x10x10 cm

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.31;
    box_pose.position.y = 0.5;
    box_pose.position.z = 0.25;
    box_pose.orientation.w = 1.0;

    box.primitives.push_back(box_primitive);
    box.primitive_poses.push_back(box_pose);
    box.operation = box.ADD;

    RCLCPP_INFO(logger, "Box aggiunta alla scena di collisione.");
    return box;
}();

// Applica la box alla scena
planning_scene_interface.applyCollisionObjects({create_box});
rclcpp::sleep_for(std::chrono::seconds(1)); // Tempo per aggiornare la scena

// (1) Creazione del publisher per PlanningScene
auto planning_scene_diff_publisher = node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);

// (2) Definisci i due punti A e B
geometry_msgs::msg::Pose pose_A;
pose_A.position.x = 0.31;
pose_A.position.y = 0.5;
pose_A.position.z = 0.25;
pose_A.orientation.w = 1.0;

geometry_msgs::msg::Pose pose_B;
pose_B.position.x = 0.31;
pose_B.position.y = -0.5;
pose_B.position.z = 0.25;
pose_B.orientation.w = 1.0;

// (3) Inizializza il clock monotono
rclcpp::Clock steady_clock(RCL_STEADY_TIME);
rclcpp::Time start_time = rclcpp::Time(0, 0, RCL_STEADY_TIME);
bool first_run = true;

// Variabile globale per salvare il tempo relativo precedente
static double previous_relative_time = 0.0;

// (4) Funzione per calcolare il tempo relativo
auto getRelativeTime = [&start_time, &steady_clock, &node]() -> double {
    return (steady_clock.now() - start_time).seconds();
};

// (5) Funzione per calcolare la posa interpolata
auto getInterpolatedPose = [](double t, const geometry_msgs::msg::Pose& start_pose, const geometry_msgs::msg::Pose& end_pose) -> geometry_msgs::msg::Pose {
    geometry_msgs::msg::Pose pose;
    pose.position.x = start_pose.position.x + (end_pose.position.x - start_pose.position.x) * t;
    pose.position.y = start_pose.position.y + (end_pose.position.y - start_pose.position.y) * t;
    pose.position.z = start_pose.position.z + (end_pose.position.z - start_pose.position.z) * t;
    pose.orientation.w = 1.0;
    return pose;
};

// (6) Durata totale del movimento e numero di cicli
double total_duration = 15.4; // Durata totale per andare da A a B (e viceversa)
int num_cycles = 1;          // Numero di cicli A -> B -> A

double cycle_duration = 2 * total_duration; // Durata di un ciclo completo (A -> B -> A)


// (7) Funzione per aggiornare la posizione della box
auto updateDynamicBox = [&](double relative_time) {
    // Approssima il tempo relativo per ridurre errori numerici
    double approx_relative_time = std::round(relative_time * 1e6) / 1e6;

    // Calcola il tempo relativo all'interno del ciclo corrente
    double cycle_relative_time = fmod(approx_relative_time, cycle_duration);

    // Determina la direzione attuale (con margine di tolleranza)
    bool current_direction = (cycle_relative_time + 0.001 < total_duration);

    // Calcola la frazione del movimento completato t
    double t = (current_direction ? cycle_relative_time : (cycle_relative_time - total_duration)) / total_duration;

    // Limita t all'intervallo [0.0, 1.0]
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;

    // Debugging: Stampa relative_time, cycle_relative_time, t e direzione
    // RCLCPP_INFO(node->get_logger(), "Relative Time: %.6f, Cycle Relative Time: %.6f, t: %.6f, Direction: %s",
                // approx_relative_time, cycle_relative_time, t, current_direction ? "A -> B" : "B -> A");

    // Calcola la posa interpolata
    geometry_msgs::msg::Pose interpolated_pose;
    if (current_direction) {
        interpolated_pose = getInterpolatedPose(t, pose_A, pose_B);
    } else {
        interpolated_pose = getInterpolatedPose(t, pose_B, pose_A);
    }

    // Debugging: Stampa la posa interpolata
    // RCLCPP_INFO(node->get_logger(), "Interpolated Pose: x=%.3f, y=%.3f, z=%.3f",
                // interpolated_pose.position.x, interpolated_pose.position.y, interpolated_pose.position.z);

    // Creazione del messaggio CollisionObject
    moveit_msgs::msg::CollisionObject move_box;
    move_box.header.frame_id = "world";
    move_box.id = "box1";
    move_box.operation = moveit_msgs::msg::CollisionObject::MOVE;
    move_box.pose = interpolated_pose;

    // Creazione del messaggio PlanningScene
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.world.collision_objects.push_back(move_box);

    // Pubblica il messaggio
    planning_scene_diff_publisher->publish(planning_scene);

    // Pubblica la posa dell'ostacolo sul topic dedicato
    geometry_msgs::msg::PoseStamped obstacle_pose_msg;
    obstacle_pose_msg.header.stamp = node->now();
    obstacle_pose_msg.header.frame_id = "world";
    obstacle_pose_msg.pose = interpolated_pose;

    obstacle_pose_publisher->publish(obstacle_pose_msg);
};

// (8) Imposta il timer per aggiornare la box
rclcpp::TimerBase::SharedPtr timer_;
timer_ = node->create_wall_timer(
        std::chrono::milliseconds(40),
        [&]() {
            if (first_run) {
            // Inizializza start_time solo al primo ciclo
            start_time = steady_clock.now();
            first_run = false;
            return;
            }

            // Calcola il tempo relativo globale
            double relative_time = getRelativeTime();

            // Aggiorna la posizione della box
            updateDynamicBox(relative_time);

            // Controlla se tutti i cicli sono stati completati
            if (relative_time >= num_cycles * cycle_duration) {
                // Interrompi il timer dopo aver completato tutti i cicli
                timer_->cancel();
                RCLCPP_INFO(node->get_logger(), "Movimento completato.");
            }
        }
    );

// Wait for the spinner thread to finish
spinner.join();
// Exit the program
return 0;
}
