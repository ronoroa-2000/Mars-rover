cmake_minimum_required(VERSION 3.8)
project(rover_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch config meshes rviz models worlds sdf
  DESTINATION share/${PROJECT_NAME}/
)






install(PROGRAMS
  scripts/virtual_differential
  scripts/waypoint_follower.py
  scripts/teleop_keyboard.py
 
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
