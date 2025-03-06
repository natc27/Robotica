cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)

# Instalar archivos URDF/Xacro
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Instalar archivos de lanzamiento
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
 
