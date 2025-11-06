cmake_minimum_required(VERSION 3.8)
project(trailblazer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# Only keep this if you actually use those msg types in code:
# find_package(ros_ign_interfaces REQUIRED)

# Install non-code assets used at runtime
install(DIRECTORY
  config launch models worlds urdf_drone
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables (must be +x and have a shebang)
install(PROGRAMS
  nodes/lidar_perception_360.py
  nodes/pose_relay.py
  nodes/altitude_lidar.py
  nodes/flight_control.py
  nodes/gui_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
