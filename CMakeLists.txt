cmake_minimum_required(VERSION 3.12)
set(BUILD_TESTING OFF CACHE BOOL "Build tests")
project(teleop_uxv_joy)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(dua_app_management REQUIRED)
find_package(dua_common_interfaces REQUIRED)
find_package(dua_hardware_interfaces REQUIRED)
find_package(dua_node_cpp REQUIRED)
find_package(dua_qos_cpp REQUIRED)
find_package(dua_uxv_interfaces REQUIRED)
find_package(params_manager_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(simple_actionclient_cpp REQUIRED)
find_package(simple_serviceclient_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)

# Generate parameters source code
generate_init_parameters(
  YAML_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/teleop_uxv_joy/params.yaml"
  OUT_FILE "init_parameters.cpp")

# Configure TeleopUXVJoy node source files
file(GLOB TELEOP_UXV_JOY_SRCS
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" CONFIGURE_DEPENDS
  "src/teleop_uxv_joy/*.cpp")
list(APPEND TELEOP_UXV_JOY_SRCS init_parameters.cpp)

# TeleopUXVJoy
add_library(teleop_uxv_joy SHARED ${TELEOP_UXV_JOY_SRCS})
target_compile_definitions(teleop_uxv_joy PRIVATE COMPOSITION_BUILDING_DLL)
target_include_directories(teleop_uxv_joy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(teleop_uxv_joy PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
target_link_libraries(teleop_uxv_joy
  Threads::Threads)
ament_target_dependencies(teleop_uxv_joy
  dua_common_interfaces
  dua_hardware_interfaces
  dua_node_cpp
  dua_qos_cpp
  dua_uxv_interfaces
  params_manager_cpp
  rclcpp
  rclcpp_components
  sensor_msgs
  simple_actionclient_cpp
  simple_serviceclient_cpp
  std_msgs
  std_srvs)
rclcpp_components_register_nodes(teleop_uxv_joy "teleop_uxv_joy::TeleopUXVJoy")

# TeleopUXVJoy standalone application
add_executable(teleop_uxv_joy_app
  src/teleop_uxv_joy_app.cpp)
target_include_directories(teleop_uxv_joy_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(teleop_uxv_joy_app PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
target_link_libraries(teleop_uxv_joy_app teleop_uxv_joy)
ament_target_dependencies(teleop_uxv_joy_app
  dua_app_management
  rclcpp)

# Install TeleopUXVJoy component
install(TARGETS teleop_uxv_joy
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install TeleopUXVJoy app
install(TARGETS teleop_uxv_joy_app
  DESTINATION lib/${PROJECT_NAME})

# Install Config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Install Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # Enable coverage instrumentation
  add_compile_options(--coverage -O0 -g)
  add_link_options(--coverage)

  # Static analysis
  find_program(CPPCHECK cppcheck)
  if(CPPCHECK)
    add_test(
      NAME cppcheck
      COMMAND ${CPPCHECK}
        --enable=warning,style,performance,portability
        --suppress=missingIncludeSystem
        --inline-suppr
        --error-exitcode=1
        -I ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  endif()
endif()

ament_package()
