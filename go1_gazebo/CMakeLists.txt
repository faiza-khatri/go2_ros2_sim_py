cmake_minimum_required(VERSION 3.8)
project(go1_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY
  meshes
  config
  xacro
  launch
  world
  maps
  models
  src
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  src/update_coordinate.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
  ff_examples_ros2/send_destination_request.py
  ff_examples_ros2/send_path_request.py
  ff_examples_ros2/send_mode_request.py
  ff_examples_ros2/teleworker_skill_dummy_action.py
  ff_examples_ros2/teleworker_skill_dummy.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
