cmake_minimum_required(VERSION 3.8)
project(image_transport_tmp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(pluginlib REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  image_transport
  pluginlib
)
set(republish republish)
add_library(${republish}_node SHARED
  src/republish.cpp
)
target_include_directories(${republish}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${republish}_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(${republish}_node ${dependencies})
rclcpp_components_register_node(${republish}_node
  PLUGIN "image_transport_tmp::Republisher"
  EXECUTABLE ${republish})

install(TARGETS
  ${republish}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY include/ DESTINATION include)
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_export_libraries(
  ${republish}_node
)
ament_package()
