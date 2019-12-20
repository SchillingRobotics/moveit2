cmake_minimum_required(VERSION 3.10.2)
project(moveit_ros_occupancy_map_monitor)
set(MOVEIT_LIB_NAME ${PROJECT_NAME})

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread)

if(APPLE)
  find_package(X11 REQUIRED)
endif(APPLE)

find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(geometric_shapes REQUIRED)

include_directories(include
                    ${Boost_INCLUDE_DIRS}
                    ${rclcpp_INCLUDE_DIRS}
                    ${moveit_core_INCLUDE_DIRS}
                    ${moveit_msgs_INCLUDE_DIRS}
                    ${geometric_shapes_INCLUDE_DIRS}
                    )
include_directories(SYSTEM
                    ${EIGEN3_INCLUDE_DIRS}
                    ${X11_INCLUDE_DIR}
                    )

add_library(${MOVEIT_LIB_NAME} SHARED
  src/occupancy_map_monitor.cpp
  src/occupancy_map_updater.cpp
  )
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME} rclcpp ${Boost_LIBRARIES} moveit_core moveit_msgs geometric_shapes)

# add_executable(moveit_ros_occupancy_map_server src/occupancy_map_server.cpp)
# ament_target_dependencies(moveit_ros_occupancy_map_server ${MOVEIT_LIB_NAME} rclcpp ${Boost_LIBRARIES})

ament_export_include_directories(include)
ament_export_interfaces(${MOVEIT_LIB_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${MOVEIT_LIB_NAME})
ament_export_dependencies(rclcpp)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(moveit_core)
ament_export_dependencies(moveit_msgs)
ament_export_dependencies(pluginlib)
ament_export_dependencies(octomap)
ament_export_dependencies(geometric_shapes)

install(
  TARGETS ${MOVEIT_LIB_NAME}
  EXPORT  ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(TARGETS moveit_ros_occupancy_map_server
#   RUNTIME DESTINATION share/${PROJECT_NAME}/
# )
install(DIRECTORY include/ DESTINATION include)

ament_package()
