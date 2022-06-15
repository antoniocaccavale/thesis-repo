cmake_minimum_required(VERSION 2.8.3)
project(gravity_compensation)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  controller_interface
  hardware_interface
  rbdl
  ddynamic_reconfigure

)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMPONENTS})



catkin_package(
  
INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_COMPONENTS}

)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${rbdl_INCLUDE_DIRS})

include_directories(
  include
)


add_library(gravity_compensation src/gravity_compensation.cpp)
target_link_libraries(gravity_compensation ${catkin_LIBRARIES})



### INSTALL ###

install(TARGETS gravity_compensation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES gravity_compensation_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/include)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

