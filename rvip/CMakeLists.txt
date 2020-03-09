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
find_package(PCL 1.8 REQUIRED)

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

add_executable(rvip src/main.cpp)
target_link_libraries(rvip ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenGR_LIBRARIES})

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

if(CATKIN_ENABLE_TESTING)
  roslint_cpp()
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)

  find_package(rostest REQUIRED)
  add_rostest_gtest(test_rvip test/run_test.test src/test/test_rvip.cpp)
  target_link_libraries(test_rvip ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenGR_LIBRARIES})

  # Create a target rvip_coverage_report
  if(ENABLE_COVERAGE_TESTING)
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage_report
      DEPENDENCIES tests
    )
  endif()

endif()
