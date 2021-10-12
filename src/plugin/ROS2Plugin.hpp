//
// Created by hans on 10/13/21.
//

#ifndef CHOREONOID_ROS2__ROS2PLUGIN_HPP_
#define CHOREONOID_ROS2__ROS2PLUGIN_HPP_

#include <cnoid/Plugin>
#include <rclcpp/rclcpp.hpp>

class ROS2Plugin : public cnoid::Plugin, public rclcpp::Node
{
public:
  ROS2Plugin();
  virtual bool initialize();
};

#endif // CHOREONOID_ROS2__ROS2PLUGIN_HPP_
