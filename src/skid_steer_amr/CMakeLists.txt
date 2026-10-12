cmake_minimum_required(VERSION 3.8)
project(skid_steer_amr)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    maps
    meshes
    rviz
    urdf
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
