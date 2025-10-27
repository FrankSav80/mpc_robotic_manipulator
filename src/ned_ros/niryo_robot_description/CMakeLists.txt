cmake_minimum_required(VERSION 3.10.2)
project(niryo_robot_description)

# Trova le dipendenze di ament
find_package(ament_cmake REQUIRED)

# Finalizza il pacchetto
ament_package()

# Installa le directory necessarie
install(DIRECTORY meshes urdf config launch worlds
  DESTINATION share/${PROJECT_NAME}
)
