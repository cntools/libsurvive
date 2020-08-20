if(OPENVR_LIBRARIES AND NOT USE_SINGLE_PRECISION)
  add_subdirectory(openvr_driver)
endif()

find_package(catkin QUIET COMPONENTS
  roscpp
  geometry_msgs)

IF(catkin_DIR)
  add_subdirectory(ros_publisher)
ENDIF()
