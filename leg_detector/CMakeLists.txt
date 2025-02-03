cmake_minimum_required(VERSION 3.8)
project(leg_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(people_msgs REQUIRED)
find_package(people_tracking_filter REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)

# Find external dependencies using PkgConfig
find_package(PkgConfig REQUIRED)
pkg_check_modules(BFL REQUIRED orocos-bfl)
pkg_check_modules(BULLET REQUIRED bullet)

# Include directories
include_directories(
  include
  ${BFL_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIRS}
)

# Declare C++ executable
add_executable(leg_detector
  src/laser_processor.cpp
  src/leg_detector.cpp
  src/calc_leg_features.cpp
)

ament_target_dependencies(leg_detector
  rclcpp
  geometry_msgs
  image_geometry
  laser_geometry
  message_filters
  people_msgs
  people_tracking_filter
  sensor_msgs
  std_msgs
  tf2
  visualization_msgs
)

# Link external libraries
target_link_libraries(leg_detector
  ${BFL_LIBRARIES}
  ${BULLET_LIBRARIES}
)

# Install the executable
install(TARGETS leg_detector
  DESTINATION lib/${PROJECT_NAME}
)

# Install configuration and launch files
install(FILES config/trained_leg_detector.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install additional resources if necessary
ament_package()
