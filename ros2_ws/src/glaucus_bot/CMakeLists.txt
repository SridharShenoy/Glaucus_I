cmake_minimum_required(VERSION 3.16)
project(glaucus_bot LANGUAGES CXX)

# Nodes that run ON THE ROBOT (Raspberry Pi). All C++.
# Logic lives in glaucus_core; these are thin ROS wrappers.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(glaucus_interfaces REQUIRED)
find_package(glaucus_core REQUIRED)
# find_package(OpenCV REQUIRED)     # TODO(glaucus): enable for vision_node
# find_package(cv_bridge REQUIRED)

# Wire protocol shared with the firmware.
set(GLAUCUS_PROTOCOL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../shared/protocol)

add_executable(sweep_executor_node src/sweep_executor_node.cpp)
target_link_libraries(sweep_executor_node glaucus_core::glaucus_core)
ament_target_dependencies(sweep_executor_node rclcpp rclcpp_action std_msgs std_srvs glaucus_interfaces)

add_executable(mcu_bridge_node src/mcu_bridge_node.cpp)
target_include_directories(mcu_bridge_node PRIVATE ${GLAUCUS_PROTOCOL_DIR})
ament_target_dependencies(mcu_bridge_node rclcpp glaucus_interfaces)

add_executable(vision_node src/vision_node.cpp)
ament_target_dependencies(vision_node rclcpp sensor_msgs glaucus_interfaces)

install(TARGETS sweep_executor_node mcu_bridge_node vision_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
