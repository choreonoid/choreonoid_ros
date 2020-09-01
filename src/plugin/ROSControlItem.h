///////////////////////////////////////////////////////////
/// @file ROSControlItem.h
/// @brief Choreonoid ControlItem Plugin for ros_control
/// @author Ryodo Tanaka 
///////////////////////////////////////////////////////////

#ifndef CNOID_ROS_PLUGIN_ROS_CONTROL_ITEM_H
#define CNOID_ROS_PLUGIN_ROS_CONTROL_ITEM_H

// Choreonoid //
#include <cnoid/ControllerItem>
#include <cnoid/Body>

// ROS //
#include <ros/ros.h>
#include "RobotHWSim.h"
#include <pluginlib/class_loader.h>
#include <controller_manager/controller_manager.h>

// STL //
#include <vector>
#include <fstream>
#include <memory>
#include <boost/shared_ptr.hpp>

namespace cnoid
{
class ROSControlItem : public ControllerItem
{
 public:
  static void initializeClass(ExtensionManager* ext);

  ROSControlItem(void);
  ROSControlItem(const ROSControlItem& org);
  
  virtual ~ROSControlItem();
  
  virtual bool initialize(ControllerIO* io) override;
  virtual bool start(void) override;
  virtual void input(void) override;
  virtual bool control(void) override;
  virtual void output(void) override;
  virtual void stop(void) override;

  virtual double timeStep() const override { return tstep_; };
  
 protected:
  virtual Item* doDuplicate(void) const;
  virtual bool store(Archive& archive);
  virtual bool restore(const Archive& archive);
  void doPutProperties(PutPropertyFunction& putProperty);
  
 private:
  ControllerIO* io_;
  Body* body_;
  double tstep_;
  double time_;

  ros::NodeHandle nh_;
  std::shared_ptr<pluginlib::ClassLoader<RobotHWSim>> rbt_hw_sim_loader_;
  RobotHWSimPtr rbt_hw_sim_;
  std::shared_ptr<controller_manager::ControllerManager> manager_;

  
  std::string namespace_{""};
  std::string robot_description_{"robot_description"};
};

typedef std::shared_ptr<ROSControlItem> ROSControlPtr;

template<class T>
boost::shared_ptr<T> to_boost(const std::shared_ptr<T> &p) {
    return boost::shared_ptr<T>(p.get(), [p](...) mutable { p.reset(); });
}

template<class T>
std::shared_ptr<T> to_std(const boost::shared_ptr<T> &p) {
    return std::shared_ptr<T>(p.get(), [p](...) mutable { p.reset(); });
}

} // namespace cnoid

#endif // CNOID_ROS_PLUGIN_ROS_CONTROL_ITEM_H
