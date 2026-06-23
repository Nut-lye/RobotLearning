cmake_minimum_required(VERSION 3.8)
project(module_c_nav_teaching)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config docs launch maps models rviz scenarios worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
