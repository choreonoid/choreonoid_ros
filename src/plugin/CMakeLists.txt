set(target CnoidROSPlugin)

choreonoid_make_gettext_mo_files(${target} mofiles)

# RobotHW inherited ros_control Choreonoid interface class #
add_library(RobotHWCnoid robot_hw_sim.h robot_hw_cnoid.cpp)
target_link_libraries(RobotHWCnoid ${catkin_LIBRARIES} Choreonoid::CnoidBody Choreonoid::CnoidBodyPlugin)

install(TARGETS RobotHWCnoid
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

# Choreonoid Plugins #
choreonoid_add_plugin(${target}
  ROSPlugin.cpp
  WorldROSItem.cpp
  BodyROSItem.cpp
  robot_hw_sim.h
  ROSControlItem.cpp
  deprecated/BodyPublisherItem.cpp
  ${mofiles}
  )

target_link_libraries(${target}
  ${roscpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${catkin_LIBRARIES}
  Choreonoid::CnoidBodyPlugin
  )

if(CHOREONOID_ENABLE_PYTHON)
  add_subdirectory(pybind11)
endif()
