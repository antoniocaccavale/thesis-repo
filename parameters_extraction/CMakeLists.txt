cmake_minimum_required(VERSION 2.8.3)
project(parameters_extraction)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  cv_bridge
  image_transport
  sensor_msgs
  control_msgs
  geometry_msgs
  gazebo_msgs
  rbdl


)

find_package(OpenCV REQUIRED 
    COMPONENTS
    opencv_highgui)



catkin_package(
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${rbdl_INCLUDE_DIRS})


add_executable(parameters_extraction src/parameters_extraction.cpp)
target_link_libraries(parameters_extraction
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}


)



### INSTALL ###

install(TARGETS parameters_extraction
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



