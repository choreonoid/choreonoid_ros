///////////////////////////////////////////////////////////
/// @file ROSControlItem.h
/// @brief Choreonoid ControlItem Plugin for ros_control
/// @author Ryodo Tanaka
///////////////////////////////////////////////////////////

#ifndef CNOID_ROS_PLUGIN_ROS_CONTROL_ITEM_H
#define CNOID_ROS_PLUGIN_ROS_CONTROL_ITEM_H

#include "RobotHWSim.h"
#include <cnoid/ControllerItem>
#include <controller_manager/controller_manager.h>
#include <pluginlib/class_loader.h>
#include <boost/shared_ptr.hpp>

namespace cnoid {

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

protected:
    virtual Item* doDuplicate() const;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    ControllerIO* io;
    double time;

    ros::NodeHandle nodeHandle;

    // boost::shared_ptr must be used to work with the ROS plugin system
    std::shared_ptr<pluginlib::ClassLoader<hardware_interface::RobotHWSim<cnoid::ControllerIO*>>> robotHWSimLoader;
    std::shared_ptr<controller_manager::ControllerManager> controllerManager;
    boost::shared_ptr<hardware_interface::RobotHWSim<cnoid::ControllerIO*>> robotHWSim;

    std::string nodeNamespace;
};

typedef ref_ptr<ROSControlItem> ROSControlItemPtr;

}  // namespace cnoid

#endif  // CNOID_ROS_PLUGIN_ROS_CONTROL_ITEM_H
