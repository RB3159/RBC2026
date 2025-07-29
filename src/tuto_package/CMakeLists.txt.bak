cmake_minimum_required(VERSION 3.5)
project(tuto_package)

# 기본 설정
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gazebo_ros REQUIRED)

# 설치할 디렉토리 설정
install(
  DIRECTORY launch urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
