cmake_minimum_required(VERSION 3.8)
project(trossen_widowx_driver_crospi)

message("--------------trossen_widowx_driver_crospi compiling ----------------")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
# crospi_core exports all its dependencies (rclcpp, pluginlib, std_msgs, sensor_msgs, geometry_msgs, ...), so they
# are found and linked through crospi_core::crospi_core. Only the Trossen specific dependencies are found here.
find_package(crospi_core REQUIRED)
find_package(libtrossen_arm REQUIRED)             # Trossen C++ driver (https://github.com/TrossenRobotics/trossen_arm)
find_package(trossen_widowx_interfaces REQUIRED)

add_library(trossen_widowx_driver_crospi SHARED
    src/trossen_widowx_driver_crospi.cpp
)

target_include_directories(trossen_widowx_driver_crospi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(trossen_widowx_driver_crospi PUBLIC
  crospi_core::crospi_core
  libtrossen_arm
  ${trossen_widowx_interfaces_TARGETS})

pluginlib_export_plugin_description_file(crospi_core plugins.xml)

ament_export_dependencies(crospi_core)

install(
  TARGETS trossen_widowx_driver_crospi
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY json_schemas
  DESTINATION share/${PROJECT_NAME})


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



ament_package()
