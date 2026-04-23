cmake_minimum_required(VERSION 3.18)
project(tracking_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenCV — must be installed on the system.
find_package(OpenCV REQUIRED)

# pybind11 — share with kinematics if already populated, else fetch.
include(FetchContent)
FetchContent_Declare(pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG        v2.12.0
  GIT_SHALLOW    TRUE)
FetchContent_MakeAvailable(pybind11)

# Core tracker static library.
add_library(tracker_core STATIC
  src/csrt_tracker.cpp
  src/optical_flow.cpp)
target_include_directories(tracker_core PUBLIC include)
target_link_libraries(tracker_core PUBLIC ${OpenCV_LIBS})

# Python extension module.
pybind11_add_module(pytracker bindings/py_tracker.cpp)
target_link_libraries(pytracker PRIVATE tracker_core)

# Output alongside kinematics module.
set(_PY_PKG_DIR "${CMAKE_SOURCE_DIR}/robot_sam2_app/robot_sam2_app")
set_target_properties(pytracker PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY         "${_PY_PKG_DIR}"
  RUNTIME_OUTPUT_DIRECTORY         "${_PY_PKG_DIR}"
  LIBRARY_OUTPUT_DIRECTORY_RELEASE "${_PY_PKG_DIR}"
  RUNTIME_OUTPUT_DIRECTORY_RELEASE "${_PY_PKG_DIR}")
