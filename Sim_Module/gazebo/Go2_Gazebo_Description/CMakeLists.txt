cmake_minimum_required(VERSION 3.8)
project(go2_gazebo_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 安装Python可执行文件
install(PROGRAMS
  go2_gazebo_description/mecanum_control.py
  go2_gazebo_description/gazebo_robot_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  go2_gazebo_description
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY
  launch
  urdf
  config
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
