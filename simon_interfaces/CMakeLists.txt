cmake_minimum_required(VERSION 3.8)
project(simon_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SimonCommand.srv"
)

# Uncomment if you have dependencies on other interfaces, e.g.
# find_package(std_msgs REQUIRED)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/SimonCommand.srv"
#   DEPENDENCIES std_msgs
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

