cmake_minimum_required(VERSION 3.16)
project(glaucus_core VERSION 0.1.0 LANGUAGES CXX)

# Pure C++17, no ROS. Builds standalone:
#   cmake -S ros2_ws/src/glaucus_core -B build/core && cmake --build build/core && ctest --test-dir build/core
# and also inside the colcon workspace (package.xml uses build_type "cmake").

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

add_library(glaucus_core
  src/sweep_controller.cpp
  src/end_of_lane_detector.cpp)
add_library(glaucus_core::glaucus_core ALIAS glaucus_core)
target_include_directories(glaucus_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(glaucus_core PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(glaucus_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

include(CTest)
if(BUILD_TESTING)
  add_executable(test_sweep_controller test/test_sweep_controller.cpp)
  target_link_libraries(test_sweep_controller glaucus_core)
  add_test(NAME sweep_controller COMMAND test_sweep_controller)

  add_executable(test_parity test/test_parity.cpp)
  target_link_libraries(test_parity glaucus_core)
  target_compile_definitions(test_parity PRIVATE
    GLAUCUS_TRACE_FILE="${CMAKE_CURRENT_SOURCE_DIR}/test/data/sim_trace_seed2.csv")
  add_test(NAME python_cpp_parity COMMAND test_parity)
endif()

# Install + export so glaucus_bot can `find_package(glaucus_core REQUIRED)`.
include(GNUInstallDirs)
install(TARGETS glaucus_core EXPORT glaucus_coreTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION include)
install(EXPORT glaucus_coreTargets NAMESPACE glaucus_core::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/glaucus_core)
include(CMakePackageConfigHelpers)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/glaucus_coreConfig.cmake
  "include(\"\${CMAKE_CURRENT_LIST_DIR}/glaucus_coreTargets.cmake\")\n")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/glaucus_coreConfig.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/glaucus_core)
