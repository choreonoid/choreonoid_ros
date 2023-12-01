set(target CnoidROS2Plugin)

choreonoid_make_gettext_mo_files(${target} mofiles)

choreonoid_add_plugin(${target}
        ROS2Plugin.cpp
        WorldROS2Item.cpp
        BodyROS2Item.cpp
        deprecated/BodyPublisherItem.cpp
        ${mofiles}
)

ament_target_dependencies(${target} PUBLIC
        "rclcpp"
        "std_msgs"
        "std_srvs"
        "sensor_msgs"
        "image_transport"
)

target_link_libraries(${target} PUBLIC
        Choreonoid::CnoidBodyPlugin
)

if (CHOREONOID_ENABLE_PYTHON)
    #  add_subdirectory(pybind11)
endif ()

install(TARGETS ${target}
        LIBRARY DESTINATION lib/${PROJECT_NAME})
