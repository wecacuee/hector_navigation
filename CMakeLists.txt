cmake_minimum_required(VERSION 2.8)
project(hector_navigation)
add_compile_options(-std=c++11)

set(CMAKE_CONFIGURATION_TYPES Release Debug RelWithDebInfo)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules")

find_package(Eigen3 REQUIRED)

add_subdirectory(hector_elevation_msgs)
add_subdirectory(hector_path_follower)
add_subdirectory(hector_exploration_planner)

add_subdirectory(hector_costmap)
add_subdirectory(hector_driving_aid_markers)
add_subdirectory(hector_elevation_mapping)
add_subdirectory(hector_elevation_visualization)
add_subdirectory(hector_exploration_controller)
add_subdirectory(hector_exploration_node)

add_subdirectory(hector_gps_calibration)
add_subdirectory(hector_timestamp_alignment)
