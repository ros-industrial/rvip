cmake_minimum_required(VERSION 2.8.3)
project(rvip)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  pcl_ros
  roscpp
  roslint
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

# DEPENDENCIES
find_package(OpenGR REQUIRED PATHS /usr/local/lib/cmake)

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

find_package(PCL REQUIRED)

add_definitions(${PCL_DEFINITIONS} )

catkin_package( CATKIN_DEPENDS
  cv_bridge
  geometry_msgs
  image_transport
  pcl_ros
  roscpp
  roslint
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenGR_LIBRARIES})

if(CATKIN_ENABLE_TESTING)

  # Downloading test_frame.pcd used for code-coverage report generation.
  catkin_download_test_data(
  ${PROJECT_NAME}_test_frame.pcd
  https://github.com/cardboardcode/test-data/raw/master/test_frame.pcd
  DESTINATION ${PROJECT_SOURCE_DIR}/test/resrc
  )

  roslint_cpp()
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)

  find_package(rostest REQUIRED)
  add_rostest_gtest(test_${PROJECT_NAME} test/run_test.test src/test/test_rvip.cpp)
  target_link_libraries(test_${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenGR_LIBRARIES})

  # Create a target rvip_coverage_report
  if(ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    # Add compiler flags for coverage instrumentation before defining any targets
    APPEND_COVERAGE_COMPILER_FLAGS()
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage_report
      DEPENDENCIES tests
    )
  endif()

endif()

# Install executable
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# Install rviz config
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
