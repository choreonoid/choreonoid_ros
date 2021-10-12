set(target CnoidROSPlugin)

choreonoid_make_gettext_mo_files(${target} mofiles)

choreonoid_add_plugin(${target}
        ROS2Plugin.cpp
        WorldROS2Item.cpp
        BodyROS2Item.cpp
  deprecated/BodyPublisherItem.cpp
  ${mofiles}
  )

target_link_libraries(${target}
  ${roscpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${image_transport_LIBRARIES}
  Choreonoid::CnoidBodyPlugin
  )

if(CHOREONOID_ENABLE_PYTHON)
  add_subdirectory(pybind11)
endif()
