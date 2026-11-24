cmake_minimum_required(VERSION 3.16)
project(g1_state_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
set(DEPENDENCIES
  rclcpp builtin_interfaces diagnostic_msgs geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros
  unitree_go unitree_hg)
foreach(dependency ${DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME}_conversions src/conversions.cpp)
target_include_directories(${PROJECT_NAME}_conversions PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_conversions ${DEPENDENCIES})

add_executable(${PROJECT_NAME} src/state_bridge_node.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_conversions)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

install(TARGETS ${PROJECT_NAME}_conversions
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_link_libraries(test_conversions ${PROJECT_NAME}_conversions)
  ament_target_dependencies(test_conversions ${DEPENDENCIES})
endif()

ament_package()
