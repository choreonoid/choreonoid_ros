set(include_dirs ${CHOREONOID_ROS_SDK_INCLUDE_DIRS})
list(REMOVE_DUPLICATES include_dirs)

set(library_dirs ${CHOREONOID_ROS_SDK_LIBRARY_DIRS})
list(REMOVE_DUPLICATES library_dirs)

set(modules
  ${CMAKE_CURRENT_BINARY_DIR}/choreonoid_rosConfig.cmake
  )

install(FILES ${modules} DESTINATION ${CHOREONOID_ROS_CMAKE_CONFIG_SUBDIR})
if(UNIX)
  string(CONCAT code
    "execute_process(COMMAND ${CMAKE_COMMAND} "
    "-E create_symlink choreonoid_rosConfig.cmake choreonoid_ros-config.cmake "
    "WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/${CHOREONOID_ROS_CMAKE_CONFIG_SUBDIR})")
  install(CODE ${code})
endif()

configure_file(ChoreonoidROSConfig.cmake.in choreonoid_rosConfig.cmake @ONLY)
