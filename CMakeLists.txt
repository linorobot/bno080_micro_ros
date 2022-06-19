cmake_minimum_required(VERSION 3.5)
project(bno080_micro_ros2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
