cmake_minimum_required(VERSION 3.18)
project(robot_project_superbuild LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(kinematics)
add_subdirectory(motor_daemon)
# add_subdirectory(tracking_cpp)  # Tier 2 — uncomment when ready
