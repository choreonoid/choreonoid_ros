#ifndef CHOREONOID_ROS__ROS2PLUGIN_HPP_
#define CHOREONOID_ROS__ROS2PLUGIN_HPP_

#include <cnoid/Plugin>
#include <rclcpp/rclcpp.hpp>

class ROS2Plugin : public cnoid::Plugin
{
public:
    ROS2Plugin();
    virtual bool initialize();
};

#endif  // CHOREONOID_ROS__ROS2PLUGIN_HPP_
