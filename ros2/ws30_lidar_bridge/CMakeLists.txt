cmake_minimum_required(VERSION 3.16)
project(ws30_lidar_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)

# ---- Locate the standalone core library ----------------------------------------
set(CORE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(CORE_INCLUDE_DIRS
  "${CORE_ROOT}/include"
  "${CORE_ROOT}/src")
find_library(RMCS_LASER_GUIDANCE_CORE_LIB
  rmcs_laser_guidance_core
  PATHS "${CORE_ROOT}/build" NO_DEFAULT_PATH)
if(NOT RMCS_LASER_GUIDANCE_CORE_LIB)
  find_library(RMCS_LASER_GUIDANCE_CORE_LIB rmcs_laser_guidance_core)
endif()
if(NOT RMCS_LASER_GUIDANCE_CORE_LIB)
  message(FATAL_ERROR "rmcs_laser_guidance_core library not found. Build the standalone core first: cmake -S . -B build -DWITH_FT4222=OFF && cmake --build build")
endif()

message(STATUS "WS30 core library: ${RMCS_LASER_GUIDANCE_CORE_LIB}")
message(STATUS "WS30 core include: ${CORE_INCLUDE_DIRS}")

# ---- Bridge node ----------------------------------------------------------------
add_executable(ws30_lidar_node
  src/ws30_lidar_node.cpp)
target_include_directories(ws30_lidar_node PRIVATE ${CORE_INCLUDE_DIRS})
target_link_libraries(ws30_lidar_node
  ${RMCS_LASER_GUIDANCE_CORE_LIB}
  ${PCL_LIBRARIES})
target_compile_definitions(ws30_lidar_node PRIVATE
  RMCS_LASER_GUIDANCE_DEFAULT_CONFIG_PATH="${CORE_ROOT}/config/default.yaml")
ament_target_dependencies(ws30_lidar_node
  rclcpp sensor_msgs diagnostic_msgs std_srvs pcl_conversions)

# ---- Install --------------------------------------------------------------------
install(TARGETS ws30_lidar_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
