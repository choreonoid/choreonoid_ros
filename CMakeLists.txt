cmake_minimum_required(VERSION 3.5.0)
project(choreonoid_ros)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE
      Release
      CACHE
        STRING
        "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
        FORCE)
endif()

find_package(
  catkin REQUIRED
  COMPONENTS angles
             choreonoid
             control_toolbox
             controller_manager
             hardware_interface
             image_transport
             joint_limits_interface
             pluginlib
             roscpp
             sensor_msgs
             std_msgs
             transmission_interface
             urdf)

catkin_package(SKIP_CMAKE_CONFIG_GENERATION SKIP_PKG_CONFIG_GENERATION)

if(CHOREONOID_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
  set(CMAKE_CXX_EXTENSIONS OFF)
else()
  add_compile_options(-std=c++14)
endif()

include_directories(${catkin_INCLUDE_DIRS})

add_subdirectory(src)

install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
