cmake_minimum_required(VERSION 3.8)
project(trailblazer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ros_ign_interfaces REQUIRED)

install(
  DIRECTORY config launch models worlds urdf_drone
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install Python nodes (executables) into lib/${PROJECT_NAME}
install(PROGRAMS
  nodes/gui_node.py
  nodes/flight_control.py 
  nodes/pose_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
