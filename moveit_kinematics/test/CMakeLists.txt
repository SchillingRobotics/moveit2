if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(moveit_resources REQUIRED)
  find_package(xmlrpcpp REQUIRED)
  find_package(moveit_ros_planning REQUIRED)

  ament_add_gtest(test_kinematics_plugin  test_kinematics_plugin.cpp
				APPEND_LIBRARY_DIRS "${append_library_dirs}"
	)

  ament_target_dependencies(
    test_kinematics_plugin
    class_loader
    pluginlib
    rclcpp
    moveit_core
    moveit_ros_planning
  )
  target_compile_definitions(test_kinematics_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

  # Run ikfast tests only if the corresponding packages were built
  find_package(fanuc_ikfast_plugin QUIET)
  if (fanuc_ikfast_plugin_FOUND)
    add_rostest(fanuc-ikfast.test ${DEPS})
  endif()

  find_package(panda_ikfast_plugin QUIET)
  if (panda_ikfast_plugin_FOUND)
    add_rostest(panda-ikfast.test ${DEPS})
  endif()

  # Benchmarking program for cached_ik_kinematics
  add_executable(benchmark_ik benchmark_ik.cpp)
  target_link_libraries(benchmark_ik
      ${catkin_LIBRARIES} ${moveit_ros_planning_LIBRARIES}
      ${Boost_PROGRAM_OPTIONS_LIBRARY})
  install(TARGETS benchmark_ik
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()