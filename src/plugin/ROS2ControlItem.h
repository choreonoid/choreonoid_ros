#ifndef CNOID_ROS_PLUGIN_ROS2_CONTROL_ITEM_H
#define CNOID_ROS_PLUGIN_ROS2_CONTROL_ITEM_H

#include "SystemInterfaceCnoid.h"

#include <cnoid/ControllerItem>

#include <controller_manager/controller_manager.hpp>
#include <rclcpp/rclcpp.hpp>


#include<chrono>
#include <memory>

namespace cnoid {

class ROS2ControlItem : public ControllerItem
{

public:
    static void initializeClass(ExtensionManager* ext);

    ROS2ControlItem();
    ROS2ControlItem(const ROS2ControlItem& org);

    virtual ~ROS2ControlItem();

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
    ControllerIO* io = nullptr;
    rclcpp::Time now;
    std::shared_ptr<rclcpp::Duration> period = nullptr;
    std::shared_ptr<rclcpp::Duration> controlPeriod;

    std::shared_ptr<rclcpp::Node> node = nullptr;
    std::shared_ptr<controller_manager::ControllerManager> controllerManager;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor = nullptr;
    std::thread executorThread;

    std::string nodeNamespace = "";
    const std::string robotDescription = "robot_description";
    std::string robotStatePublisherName = "robot_state_publisher";

    std::string getURDF() const;
};

typedef ref_ptr<ROS2ControlItem> ROS2ControlItemPtr;

}  // namespace cnoid

#endif  // CNOID_ROS_PLUGIN_ROS2_CONTROL_ITEM_H
