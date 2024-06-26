#ifndef CNOID_ROS_PLUGIN_ROS2_PLUGIN_HPP_
#define CNOID_ROS_PLUGIN_ROS2_PLUGIN_HPP_

#include <cnoid/Plugin>
#include <rclcpp/rclcpp.hpp>

class ROS2Plugin : public cnoid::Plugin
{
public:
    ROS2Plugin();
    virtual bool customizeApplication(cnoid::AppCustomizationUtil& app) override;
    virtual bool initialize() override;
};

#endif  // CNOID_ROS_PLUGIN_ROS2_PLUGIN_HPP_
