cmake_minimum_required(VERSION 3.8)
project(my_robot_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gazebo_ros REQUIRED)

# 노드 빌드가 필요하면 아래 부분 추가 (필요 없으면 생략)
# add_executable(key_control src/key_control.cpp)
# ament_target_dependencies(key_control
#   rclcpp
#   std_msgs
#   urdf
# )
# install(TARGETS
#   key_control
#   DESTINATION lib/${PROJECT_NAME}
# )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

