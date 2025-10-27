cmake_minimum_required(VERSION 3.22)
project(niryo_moveit2_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(Eigen3 REQUIRED)

# Aggiungere file C++
add_executable(add_boxes cpp_script/add_boxes.cpp)
add_executable(dynamic_scene_setup cpp_script/dynamic_scene_setup.cpp)
add_executable(lin_traj_chomp cpp_script/lin_traj_chomp.cpp)
add_executable(mpc_controller cpp_script/mpc_controller.cpp)               # Aggiuto questo

# Configurazione manuale di CasADi e IPOPT
target_include_directories(mpc_controller PRIVATE
    /usr/local/include/casadi  # Percorso degli header di CasADi
    /usr/include/coin          # Percorso degli header di IPOPT
)

# Link delle librerie di CasADi e IPOPT
target_link_libraries(mpc_controller
    /usr/local/lib/libcasadi.so               # Libreria principale di CasADi
    /usr/local/lib/libcasadi_nlpsol_ipopt.so  # Plugin di CasADi per IPOPT
    /lib/libipopt.so                          # Libreria principale di IPOPT
)

# Dipendenze necessarie per 'add_boxes'
ament_target_dependencies(
  add_boxes
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  shape_msgs
  geometry_msgs
  moveit_ros_move_group
)

# Dipendenze necessarie per 'dynamic_scene_setup'
ament_target_dependencies(
  dynamic_scene_setup
  rclcpp
  moveit_visual_tools
  geometry_msgs
  moveit_ros_planning_interface
  moveit_ros_move_group
)

# Dipendenze necessarie per 'lin_traj_chomp'
ament_target_dependencies(
  lin_traj_chomp
  rclcpp
  moveit_visual_tools
  moveit_ros_planning_interface
  geometry_msgs
  moveit_ros_move_group
)

# Dipendenze necessarie per 'mpc_controller'        Aggiuto questo
ament_target_dependencies(
  mpc_controller
  rclcpp
  moveit_visual_tools
  geometry_msgs
  moveit_ros_planning_interface
  moveit_ros_move_group
  std_msgs
  Eigen3
)

# Installa gli eseguibili
install(TARGETS add_boxes dynamic_scene_setup lin_traj_chomp mpc_controller        # Aggiuto questo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
