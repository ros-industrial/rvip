cmake_minimum_required(VERSION 2.8.3)
project(rvip_roi_parser)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  darknet_ros_msgs
  roscpp
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  ROIArray.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package( CATKIN_DEPENDS
  darknet_ros_msgs
  roscpp
  sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} rvip_roi_parser_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

# Install executable
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
