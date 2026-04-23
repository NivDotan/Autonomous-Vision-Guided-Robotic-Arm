cmake_minimum_required(VERSION 3.18)
project(motor_daemon LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ─────────────────────────────────────────────────────────────
include(FetchContent)

# ZeroMQ — check system first, then build from source as fallback.
find_package(ZeroMQ QUIET)
if(NOT ZeroMQ_FOUND)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(ZMQ libzmq)
  endif()
endif()

if(NOT ZMQ_FOUND AND NOT ZeroMQ_FOUND)
  message(STATUS "ZeroMQ not found via pkg-config, fetching from source...")
  FetchContent_Declare(zeromq
    GIT_REPOSITORY https://github.com/zeromq/libzmq.git
    GIT_TAG        v4.3.5
    GIT_SHALLOW    TRUE)
  set(BUILD_TESTS        OFF CACHE BOOL "" FORCE)
  set(WITH_DOCS          OFF CACHE BOOL "" FORCE)
  set(ENABLE_DRAFTS      OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(zeromq)
  set(ZMQ_TARGET libzmq)
else()
  set(ZMQ_TARGET ${ZeroMQ_LIBRARIES})
endif()

# msgpack-cxx — header-only C++ MessagePack serialization
FetchContent_Declare(msgpack
  GIT_REPOSITORY https://github.com/msgpack/msgpack-c.git
  GIT_TAG        cpp-3.3.0
  GIT_SHALLOW    TRUE)
set(MSGPACK_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(MSGPACK_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(msgpack)

# ── Executable ───────────────────────────────────────────────────────────────
add_executable(motor_daemon
  src/motor_daemon.cpp
  src/pid_controller.cpp
  src/trajectory_tracker.cpp
  src/serial_comm.cpp)

target_include_directories(motor_daemon PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${msgpack_SOURCE_DIR}/include)

target_link_libraries(motor_daemon PRIVATE ${ZMQ_TARGET})

if(WIN32)
  # WinSock needed by ZeroMQ on Windows.
  target_link_libraries(motor_daemon PRIVATE ws2_32 iphlpapi)
endif()

if(UNIX)
  target_link_libraries(motor_daemon PRIVATE pthread)
endif()

# Output to project root for easy invocation.
set_target_properties(motor_daemon PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY         "${CMAKE_SOURCE_DIR}"
  RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}")

# Copy ZeroMQ DLL next to the executable.
add_custom_command(TARGET motor_daemon POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:libzmq>
    "${CMAKE_SOURCE_DIR}/$<TARGET_FILE_NAME:libzmq>")
