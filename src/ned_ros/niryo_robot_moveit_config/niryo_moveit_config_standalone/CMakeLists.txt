cmake_minimum_required(VERSION 3.10.2)
project(niryo_moveit_config_standalone)

find_package(ament_cmake REQUIRED)
find_package(niryo_robot_description REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
