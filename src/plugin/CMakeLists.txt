set(target CnoidROSPlugin)

choreonoid_make_gettext_mo_files(${target} mofiles)

choreonoid_add_plugin(${target}
  ROSPlugin.cpp
  BodyPublisherItem.cpp
  ${mofiles}
  )

target_link_libraries(${target}
  ${roscpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${CHOREONOID_BASE_LIBRARIES}
  CnoidROSUtil
  # Qt5::Widgets
  )

if(CHOREONOID_ENABLE_PYTHON)
  add_subdirectory(pybind11)
endif()
