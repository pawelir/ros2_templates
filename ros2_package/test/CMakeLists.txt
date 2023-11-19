# Interface Adapter node test
ament_add_gtest(ros2_package_node_test ros2_package_node_test.cpp)
ament_target_dependencies(ros2_package_node_test
  ${dependencies}
)
target_link_libraries(ros2_package_node_test
  ${library_name}
)
