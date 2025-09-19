cmake_minimum_required(VERSION 3.8)
project(range_sensor_multi_zone)

# Generate compile_commands.json for IntelliSense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2 -g)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(range_sensor_multi_zone
    src/platform/linux_platform.cpp
    src/range_sensor_multi_zone.cpp
    src/vl53l5cx_api.cpp
    src/vl53l5cx_plugin_detection_thresholds.cpp
    src/vl53l5cx_plugin_motion_indicator.cpp
    src/vl53l5cx_plugin_xtalk.cpp)

target_include_directories(range_sensor_multi_zone PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(range_sensor_multi_zone PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  range_sensor_multi_zone
  "rclcpp"
  "sensor_msgs"
)

install(TARGETS range_sensor_multi_zone
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
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

ament_package()
