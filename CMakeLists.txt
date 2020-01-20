cmake_minimum_required(VERSION 3.5.0)
project(choreonoid_ros)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  image_transport
  choreonoid
  )

# find_package(Qt5 COMPONENTS Widgets REQUIRED)

catkin_package(SKIP_CMAKE_CONFIG_GENERATION SKIP_PKG_CONFIG_GENERATION)

if(CHOREONOID_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
  set(CMAKE_CXX_EXTENSIONS OFF)
else()
  add_compile_options(-std=c++14)
endif()

add_definitions(${CHOREONOID_DEFINITIONS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
)

link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
  )

add_subdirectory(src)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
