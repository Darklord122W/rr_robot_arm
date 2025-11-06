cmake_minimum_required(VERSION 3.8)
project(rr_arm)

find_package(ament_cmake REQUIRED)

# Just install your resources; no need to find robot/xacro/jsp here.
install(DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()