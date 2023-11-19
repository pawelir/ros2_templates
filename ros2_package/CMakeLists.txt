cmake_minimum_required(VERSION 3.5)
project(ros2_package)

### Dependencies ###

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED) # Replace with your package's API dependencies
find_package(std_srvs REQUIRED) # Replace with your package's API dependencies

set(dependencies
  ament_cmake
  rclcpp
  std_msgs # Replace with your package's API dependencies
  std_srvs # Replace with your package's API dependencies
)

### Libraries and executables ###

include_directories(
  include
)

set(executable_name ros2_package)
set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/ros2_package_node.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name}
  ${library_name}
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

### Install ###

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

### Test ###

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_clang_format
  )
  ament_execute_extensions(ament_lint_auto EXCLUDE ${AMENT_LINT_AUTO_EXCLUDE})
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE /ros_ws/src/.clang-format)
endif()

### Export ###

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
