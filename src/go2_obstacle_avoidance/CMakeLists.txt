cmake_minimum_required(VERSION 3.8)
project(go2_obstacle_avoidance)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/obstacle_avoidance_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
