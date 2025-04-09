cmake_minimum_required(VERSION 3.5.0)
project(choreonoid_ros)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()

if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 1)
    find_package(
      catkin REQUIRED COMPONENTS
      roscpp
      angles
      joint_limits_interface
      transmission_interface
      urdf
    )
  elseif ($ENV{ROS_VERSION} EQUAL 2)
    find_package(
      ament_cmake REQUIRED COMPONENTS
      rclcpp
      rclcpp_lifecycle
      ament_index_cpp
    )
  endif()
endif()

find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(choreonoid REQUIRED)

if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 1)
    catkin_package(SKIP_CMAKE_CONFIG_GENERATION SKIP_PKG_CONFIG_GENERATION)
    include_directories(${catkin_INCLUDE_DIRS})
  endif()
endif()

if(CHOREONOID_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
  set(CMAKE_CXX_EXTENSIONS OFF)
else()
  add_compile_options(-std=c++14)
endif()


add_subdirectory(src)

if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 1)
    # For ros_control
    install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

    # Install `launch` dir. only if `--install` option is set in catkin config
    if(${CMAKE_INSTALL_PREFIX} MATCHES .*/install)
      install(DIRECTORY launch
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
          USE_SOURCE_PERMISSIONS
          PATTERN "ros2" EXCLUDE
        )
    endif()
  elseif($ENV{ROS_VERSION} EQUAL 2)
    pluginlib_export_plugin_description_file(hardware_interface choreonoid_ros2_control.xml)

    install(DIRECTORY
      launch/ros2
      DESTINATION share/${PROJECT_NAME}/launch
    )

    ament_package()
  endif()
endif()
