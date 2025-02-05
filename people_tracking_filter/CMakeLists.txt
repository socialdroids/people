cmake_minimum_required(VERSION 3.8)
project(people_tracking_filter)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
  # add_compile_options(-Wall -Wextra -Wpedantic) # Muitos warnings devido ao -Wpedantic
endif()

find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(people_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

if (EXISTS "FindEigen3.cmake")
  find_package(Eigen3 REQUIRED)
  set(Eigen_INCLUDE_DIRS ${Eigen3_INCLUDE_DIRS})
elseif (EXISTS "FindEigen.cmake")
  find_package(Eigen REQUIRED)
elseif (EXISTS "FindEigen.cmake")
  message(WARNING "No findEigen cmake script found. You must provde one of them,
  e.g. by adding it to ${PROJECT_SOURCE_DIR}/cmake_modules.")
endif (EXISTS "FindEigen3.cmake")

set(EXTERNAL_INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${BFL_INCLUDE_DIRS})

include_directories(${BFL_INCLUDE_DIRS}/bfl)
link_directories(${BFL_LIBRARY_DIRS})
include_directories(${tf2_INCLUDE_DIRS})
# Declare the libraries and executable

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EXTERNAL_INCLUDE_DIRS}
  ${BFL_INCLUDE_DIRS}
)

add_library(people_tracking_filter SHARED
  src/uniform_vector.cpp
  src/gaussian_vector.cpp
  src/gaussian_pos_vel.cpp
  src/mcpdf_pos_vel.cpp
  src/mcpdf_vector.cpp
  src/sysmodel_pos_vel.cpp
  src/sysmodel_vector.cpp
  src/measmodel_pos.cpp
  src/measmodel_vector.cpp
  src/tracker_particle.cpp
  src/tracker_kalman.cpp
  src/detector_particle.cpp
)

set(ament_dependencies
  rclcpp
  geometry_msgs
  tf2
  people_msgs
  message_filters
  sensor_msgs
  tf2_ros
  pcl_ros
)


message("BFL include dirs:" ${BFL_INCLUDE_DIRS})
message("BFL library dirs:" ${BFL_LIBRARY_DIRS})

ament_target_dependencies(people_tracking_filter
   ${ament_dependencies}
)

target_link_libraries(people_tracking_filter orocos-bfl
  ${Boost_LIBRARIES} ${BFL_LIBRARIES}
)

# Create the executable for the people_tracker node
add_executable(people_tracker src/people_tracking_node.cpp)
ament_target_dependencies(people_tracker ${ament_dependencies})
target_link_libraries(people_tracker orocos-bfl
  ${Boost_LIBRARIES} ${BFL_LIBRARIES} people_tracking_filter
)


# Install the targets
install(TARGETS people_tracking_filter
  DESTINATION lib/
)
install(TARGETS people_tracker
  DESTINATION lib/
)


install(DIRECTORY include/
  DESTINATION include/
)


ament_export_include_directories(include)
ament_export_libraries(people_tracking_filter)
ament_export_dependencies(${ament_dependencies})

# Ament package macros
ament_package()
