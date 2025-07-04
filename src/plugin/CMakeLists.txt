set(target CnoidROSPlugin)

choreonoid_make_gettext_mo_files(${target} mofiles)

if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 1)
    set(sources
      ROSPlugin.cpp
      WorldROSItem.cpp
      BodyROSItem.cpp
      ROSControlItem.cpp
      RobotHWSim.h
      deprecated/BodyPublisherItem.cpp
    )

    set(headers
      WorldROSItem.h
      BodyROSItem.h
      exportdecl.h
    )

    choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})

    target_link_libraries(${target}
      PUBLIC
      ${roscpp_LIBRARIES}
      ${std_msgs_LIBRARIES}
      ${sensor_msgs_LIBRARIES}
      ${image_transport_LIBRARIES}
      ${catkin_LIBRARIES}
      Choreonoid::CnoidBodyPlugin
    )

    # PointCloud(1) is deprecated in ROS 2, but it is still used in ROS 1
    option(USE_POINTCLOUD1_IN_BODY_ROS_ITEM "Use the PointCloud type instead of PointCloud2 for publishing range sensor data with ROS 1" OFF)
    mark_as_advanced(USE_POINTCLOUD1_IN_BODY_ROS_ITEM)

    if(USE_POINTCLOUD1_IN_BODY_ROS_ITEM)
      set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS CNOID_ROS_PLUGIN_USE_POINTCLOUD1)
    endif()

    # RobotHW inherited ros_control Choreonoid interface class
    add_library(RobotHWCnoid RobotHWSim.h RobotHWCnoid.cpp)
    target_link_libraries(RobotHWCnoid ${catkin_LIBRARIES} Choreonoid::CnoidBody Choreonoid::CnoidBase)

    install(TARGETS RobotHWCnoid
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )

    if(CHOREONOID_ENABLE_PYTHON)
      add_subdirectory(pybind11)
    endif()
  elseif($ENV{ROS_VERSION} EQUAL 2)
    set(sources
      ROS2Plugin.cpp
      WorldROS2Item.cpp
      BodyROS2Item.cpp
    )

    set(headers
      WorldROS2Item.h
      BodyROS2Item.h
      exportdecl.h
    )

    choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})

    ament_target_dependencies(${target} PUBLIC
      "rclcpp"
      "std_msgs"
      "std_srvs"
      "sensor_msgs"
      "image_transport"
    )

    target_link_libraries(${target} PUBLIC
      Choreonoid::CnoidBodyPlugin
    )

    if(CHOREONOID_ENABLE_PYTHON)
      #  add_subdirectory(pybind11)
    endif ()

  endif()
endif()
