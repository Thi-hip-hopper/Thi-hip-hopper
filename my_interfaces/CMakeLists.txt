cmake_minimum_required(VERSION 3.5)
project(my_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)  # <<< Important!

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RectangleDimensions.srv"
  DEPENDENCIES builtin_interfaces
)

ament_package()

