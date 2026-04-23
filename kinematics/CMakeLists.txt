cmake_minimum_required(VERSION 3.18)
project(kinematics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies via FetchContent ───────────────────────────────────────────
include(FetchContent)

# Eigen 3.4.0 — header-only linear algebra
FetchContent_Declare(Eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.0
  GIT_SHALLOW    TRUE
  GIT_PROGRESS   TRUE)
set(EIGEN_BUILD_DOC      OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING        OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING  OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(Eigen)

# nlohmann/json — single header for calibration JSON loading
FetchContent_Declare(nlohmann_json
  URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.hpp
  DOWNLOAD_NO_EXTRACT TRUE
  DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/include/nlohmann)
FetchContent_MakeAvailable(nlohmann_json)

# pybind11 v2.12.0 — Python bindings
FetchContent_Declare(pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG        v2.12.0
  GIT_SHALLOW    TRUE)
FetchContent_MakeAvailable(pybind11)

# ── Core static library ─────────────────────────────────────────────────────
add_library(kinematics_core STATIC
  src/dh_kinematics.cpp
  src/ik_solver.cpp
  src/jacobian.cpp
  src/trajectory.cpp)

target_include_directories(kinematics_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/include   # for nlohmann/json.hpp
  ${eigen_SOURCE_DIR})

# ── Python extension module ─────────────────────────────────────────────────
pybind11_add_module(pykinematics bindings/py_kinematics.cpp)
target_link_libraries(pykinematics PRIVATE kinematics_core)

# Place the .pyd / .so directly into the Python package so `import pykinematics` works
set(_PY_PKG_DIR "${CMAKE_SOURCE_DIR}/robot_sam2_app/robot_sam2_app")
set_target_properties(pykinematics PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY         "${_PY_PKG_DIR}"
  RUNTIME_OUTPUT_DIRECTORY         "${_PY_PKG_DIR}"
  LIBRARY_OUTPUT_DIRECTORY_RELEASE "${_PY_PKG_DIR}"
  RUNTIME_OUTPUT_DIRECTORY_RELEASE "${_PY_PKG_DIR}")

# ── Unit tests ──────────────────────────────────────────────────────────────
enable_testing()
add_executable(test_kinematics tests/test_kinematics.cpp)
target_link_libraries(test_kinematics PRIVATE kinematics_core)
target_include_directories(test_kinematics PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/include
  ${eigen_SOURCE_DIR})
add_test(NAME kinematics_unit COMMAND test_kinematics)
