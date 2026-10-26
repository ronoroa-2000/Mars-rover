cmake_minimum_required(VERSION 3.8)
project(navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz config maps
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS scripts/ndt_ekf_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
