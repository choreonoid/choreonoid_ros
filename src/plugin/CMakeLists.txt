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

if(CHOREONOID_ENABLE_PYTHON)
  add_subdirectory(pybind11)
endif()
