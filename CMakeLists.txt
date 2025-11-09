cmake_minimum_required(VERSION 3.8)
project(trailblazer)

# ---------------------------
# Compiler hygiene
# ---------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------
# Find ament + Python helpers
# ---------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# If you use custom ROS messages/services/actions, find them here:
# find_package(ros_ign_interfaces REQUIRED)

# ---------------------------
# Install data assets
# These are accessed at runtime by launch files (configs, models, worlds, URDF).
# ---------------------------
install(DIRECTORY
  config
  launch
  models
  worlds
  urdf_drone
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------
# Install Python utility package (importable as `trailblazer_utils`)
# This is where `map_printer.py` now lives.
#   trailblazer_utils/
#     __init__.py
#     map_printer.py
# ---------------------------
ament_python_install_package(trailblazer_utils)

# ---------------------------
# Install Python node entry points
# (Ensure each file has a proper shebang and chmod +x)
# ---------------------------
install(PROGRAMS
  nodes/pose_relay.py
  nodes/altitude_lidar.py
  nodes/flight_control.py
  nodes/gui_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------
# Package export
# ---------------------------
ament_package()
