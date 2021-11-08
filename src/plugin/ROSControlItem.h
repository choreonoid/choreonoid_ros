///////////////////////////////////////////////////////////
/// @file ROSControlItem.h
/// @brief Choreonoid ControlItem Plugin for ros_control
/// @author Ryodo Tanaka
///////////////////////////////////////////////////////////

#ifndef CNOID_ROS_PLUGIN_ROS_CONTROL_ITEM_H
#define CNOID_ROS_PLUGIN_ROS_CONTROL_ITEM_H

// Choreonoid //
#include <cnoid/Body>
#include <cnoid/ControllerItem>

// ROS //
#include <controller_manager/controller_manager.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include "robot_hw_sim.h"

// STL //
#include <fstream>
#include <vector>

// Boost //
#include <boost/shared_ptr.hpp>

namespace cnoid
{
class ROSControlItem : public ControllerItem
{
public:
  static void initializeClass(ExtensionManager* ext);

  ROSControlItem();
  ROSControlItem(const ROSControlItem& org);

  virtual ~ROSControlItem();

  virtual bool initialize(ControllerIO* io) override;
  virtual bool start() override;
  virtual void input() override;
  virtual bool control() override;
  virtual void output() override;
  virtual void stop() override;

  virtual double timeStep() const override
  {
    return tstep_;
  };

protected:
  virtual Item* doDuplicate() const;
  virtual bool store(Archive& archive);
  virtual bool restore(const Archive& archive);
  void doPutProperties(PutPropertyFunction& putProperty);

private:
  ControllerIO* io_;
  Body* body_;
  double tstep_;
  double time_;

  ros::NodeHandle nh_;
  std::shared_ptr<pluginlib::ClassLoader<hardware_interface::RobotHWSim<cnoid::ControllerIO*>>> rbt_hw_sim_loader_;
  std::shared_ptr<controller_manager::ControllerManager> manager_;

  // we have to use boost::shared_ptr since ROS plugin system uses it
  boost::shared_ptr<hardware_interface::RobotHWSim<cnoid::ControllerIO*>> rbt_hw_sim_;

  std::string namespace_{ "" };
};

typedef ref_ptr<ROSControlItem> ROSControlItemPtr;

}  // namespace cnoid

#endif  // CNOID_ROS_PLUGIN_ROS_CONTROL_ITEM_H
