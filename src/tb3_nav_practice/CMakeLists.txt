cmake_minimum_required(VERSION 3.8)
project(tb3_nav_practice)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
