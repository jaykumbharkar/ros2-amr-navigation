cmake_minimum_required(VERSION 3.8)
project(new_robot)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    urdf
    meshes
    worlds
    config
    maps
    rviz
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
