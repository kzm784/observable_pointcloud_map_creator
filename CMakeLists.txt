cmake_minimum_required(VERSION 3.8)
project(observable_pointcloud_map_creator)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(observable_pointcloud_map_creator
  SHARED
    src/observable_pointcloud_map_creator.cpp
)
rclcpp_components_register_nodes(observable_pointcloud_map_creator
  PLUGIN "ObservablePointcloudMapCreator"
)

ament_auto_add_executable(observable_pointcloud_map_creator_node
  src/observable_pointcloud_map_creator_node.cpp
)
target_link_libraries(observable_pointcloud_map_creator_node
  observable_pointcloud_map_creator
)
install(TARGETS observable_pointcloud_map_creator_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)
