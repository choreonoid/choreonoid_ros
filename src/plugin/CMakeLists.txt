set(target CnoidROSPlugin)

cnoid_make_gettext_mofiles(${target} mofiles)

add_cnoid_plugin(${target} SHARED
  ROSPlugin.cpp
  BodyPublisherItem.cpp
  ${mofiles}
  )

target_link_libraries(${target}
  ${roscpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${CHOREONOID_LIBRARIES}
  CnoidROSUtil
  # Qt5::Widgets
  )

set(plugin_dir ${CATKIN_PACKAGE_BIN_DESTINATION}/${CHOREONOID_VERSION_SUBDIR})
set_target_properties(${target} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${plugin_dir})
install(TARGETS ${target} LIBRARY DESTINATION ${plugin_dir})

if(CHOREONOID_ENABLE_PYTHON)
  if(CHOREONOID_USE_PYBIND11)
    add_subdirectory(pybind11)
  endif()
endif()
