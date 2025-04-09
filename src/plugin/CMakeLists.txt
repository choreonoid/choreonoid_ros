set(target CnoidROSPlugin)

choreonoid_make_gettext_mo_files(${target} mofiles)

if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 1)
    choreonoid_add_plugin(${target}
      ROSPlugin.cpp
      WorldROSItem.cpp
      BodyROSItem.cpp
      ROSControlItem.cpp
      RobotHWSim.h
      deprecated/BodyPublisherItem.cpp
      ${mofiles}
    )

    target_link_libraries(${target}
      PUBLIC
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
    choreonoid_add_plugin(${target}
      ROS2Plugin.cpp
      WorldROS2Item.cpp
      BodyROS2Item.cpp
      ROS2ControlItem.cpp
      SystemInterfaceCnoid.cpp
      ${mofiles}
    )

    ament_target_dependencies(${target} PUBLIC
      "rclcpp"
      "std_msgs"
      "std_srvs"
      "sensor_msgs"
      "image_transport"
      "hardware_interface"
      "controller_manager"
    )

    target_link_libraries(${target} PUBLIC
      Choreonoid::CnoidBodyPlugin
    )

    add_library(choreonoid_ros2_control SHARED SystemInterfaceCnoid.cpp)
    target_link_libraries(choreonoid_ros2_control PUBLIC Choreonoid::CnoidBody Choreonoid::CnoidBase)
    ament_target_dependencies(choreonoid_ros2_control PUBLIC
      controller_manager
      hardware_interface
      pluginlib
      rclcpp
      rclcpp_lifecycle
    )

    install(TARGETS choreonoid_ros2_control
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib
    )

    ament_export_libraries(choreonoid_ros2_control)
    ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

    if(CHOREONOID_ENABLE_PYTHON)
      #  add_subdirectory(pybind11)
    endif ()

  endif()
endif()
