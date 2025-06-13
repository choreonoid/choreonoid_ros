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
      pluginlib
      angles
      controller_manager
      hardware_interface
      joint_limits_interface
      transmission_interface
      urdf
    )
  elseif ($ENV{ROS_VERSION} EQUAL 2)
    find_package(
      ament_cmake REQUIRED COMPONENTS
      rclcpp ament_index_cpp
    )
  endif()
endif()

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(choreonoid REQUIRED)

set(CHOREONOID_ROS_CMAKE_CONFIG_SUBDIR share/choreonoid_ros/cmake) # ROS compliant

set(CHOREONOID_ROS_SDK_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/${CHOREONOID_HEADER_SUBDIR})
set(CHOREONOID_ROS_SDK_LIBRARY_DIRS
  ${CMAKE_INSTALL_PREFIX}/${CHOREONOID_LIB_SUBDIR} ${CMAKE_INSTALL_PREFIX}/${CHOREONOID_PLUGIN_SUBDIR})

set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS $<$<CONFIG:Debug>:CNOID_DEBUG>)

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

if(NOT PROJECT_BINARY_DIR STREQUAL PROJECT_SOURCE_DIR)
  include_directories(${PROJECT_BINARY_DIR})
  include_directories(${PROJECT_BINARY_DIR}/include)
endif()
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)

link_directories(${PROJECT_BINARY_DIR}/lib)
link_directories(${PROJECT_BINARY_DIR}/${CHOREONOID_PLUGIN_SUBDIR})

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
    ament_package()

    install(DIRECTORY
      launch/ros2
      DESTINATION share/${PROJECT_NAME}/launch
    )
  endif()
endif()

add_subdirectory(include)
add_subdirectory(cmake)
