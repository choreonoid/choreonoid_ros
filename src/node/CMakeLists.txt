set(target choreonoid)
add_executable(${target} choreonoid_ros2.cpp)
ament_target_dependencies(${target} rclcpp std_msgs sensor_msgs image_transport)
set_target_properties(${target} PROPERTIES OUTPUT_NAME choreonoid)
target_link_libraries(${target} Choreonoid::CnoidBase  ${rclcpp_LIBRARIES})
install(TARGETS ${target}
        RUNTIME DESTINATION lib/${PROJECT_NAME})
