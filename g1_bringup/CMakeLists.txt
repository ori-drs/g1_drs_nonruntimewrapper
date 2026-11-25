cmake_minimum_required(VERSION 3.16)
project(g1_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/record_bag.py DESTINATION lib/${PROJECT_NAME})

ament_package()
