cmake_minimum_required(VERSION 3.8)
project(trossen_widowx_driver_crospi)

message("--------------trossen_widowx_driver_crospi compiling ----------------")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_ros REQUIRED)
# find_package(ament_index_cpp REQUIRED)

find_package(pluginlib REQUIRED)
find_package(crospi_core REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trossen_widowx_interfaces REQUIRED)

# Find the libtrossen_arm package
find_package(libtrossen_arm REQUIRED)
find_package(trossen_widowx_interfaces REQUIRED)

# -- start --Stuff for triple buffer implementation of boost 1.89
if(DEFINED ENV{BOOST_189_ROOT} AND NOT "$ENV{BOOST_189_ROOT}" STREQUAL "")
  set(_BOOST_1_89_CUSTOM_ROOT "$ENV{BOOST_189_ROOT}")
  message(STATUS "Using Boost 1.89 from BOOST_189_ROOT=${_BOOST_1_89_CUSTOM_ROOT}")
else()
  if(DEFINED ENV{HOME} AND NOT "$ENV{HOME}" STREQUAL "")
    set(_BOOST_1_89_CUSTOM_ROOT "$ENV{HOME}/.local/lib/boost/1.89.0")
    message(STATUS "Using Boost 1.89 from HOME fallback: ${_BOOST_1_89_CUSTOM_ROOT}")
  else()
    message(FATAL_ERROR
      "Boost 1.89 requested, but neither BOOST_189_ROOT nor HOME is defined")
  endif()
endif()

# --------------------------------------------------
# Find Boost (headers-only or full, as needed)
# --------------------------------------------------
# find_package(Boost 1.89 REQUIRED
#   HINTS
#     ${_BOOST_1_89_CUSTOM_ROOT}
#   PATHS
#     ${_BOOST_1_89_CUSTOM_ROOT}
# )

# Optional but useful for debugging


set(BOOST_ROOT "${_BOOST_1_89_CUSTOM_ROOT}")
set(Boost_INCLUDE_DIR "${_BOOST_1_89_CUSTOM_ROOT}/include")
message(STATUS "CMAKE_PREFIX_PATH before: ${CMAKE_PREFIX_PATH}")
set(CMAKE_PREFIX_PATH "${_BOOST_1_89_CUSTOM_ROOT}" ${CMAKE_PREFIX_PATH})
message(STATUS "CMAKE_PREFIX_PATH after: ${CMAKE_PREFIX_PATH}")
message(STATUS "Using Boost version ${Boost_VERSION}")

find_package(Boost 1.89 REQUIRED)
message(STATUS "Boost include dirs after find_package: ${Boost_INCLUDE_DIR}")
# -- end -- Stuff for triple buffer implementation of boost 1.89

include_directories(
   include
)

set(CMAKE_CXX_STANDARD 17)

# include_directories(include ${LUA_INCLUDE_DIR})

set(src_trossen_widowx_driver_crospi
    src/trossen_widowx_driver_crospi.cpp
)

add_library(trossen_widowx_driver_crospi 
            SHARED
            ${src_trossen_widowx_driver_crospi})

set(MY_EXTERNAL_LIB "") #sets it empty such that it can be redefined later to include external lib


set(rosdependencies
  "crospi_core"
  "pluginlib"
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "trossen_widowx_interfaces"
)


set(dependencies
    fmt   
    jsoncpp
    libtrossen_arm
  )

pluginlib_export_plugin_description_file(crospi_core plugins.xml)


target_include_directories(trossen_widowx_driver_crospi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS})

ament_target_dependencies(trossen_widowx_driver_crospi ${rosdependencies})  
target_link_libraries(trossen_widowx_driver_crospi ${dependencies})

ament_export_dependencies(crospi_core)

ament_export_libraries(
  trossen_widowx_driver_crospi
)

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
