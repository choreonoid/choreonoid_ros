/**
   \file
   \author Yuki Onishi
*/

#ifndef CNOID_ROS_PLUGIN_LINK_VELOCITY_CONTROLLER_ROS_ITEM_H
#define CNOID_ROS_PLUGIN_LINK_VELOCITY_CONTROLLER_ROS_ITEM_H

#include <cnoid/Archive>
#include <cnoid/ControllerItem>
#include <cnoid/Link>
#include <cnoid/PutPropertyFunction>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <ostream>
#include <string>

#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT LinkVelocityControllerROSItem : public ControllerItem
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    LinkVelocityControllerROSItem();
    LinkVelocityControllerROSItem(const LinkVelocityControllerROSItem& org);
    virtual ~LinkVelocityControllerROSItem();
    
    virtual bool initialize(ControllerIO* io) override;
    virtual bool start() override;
    virtual void input() override;
    virtual bool control() override;
    virtual void output() override;
    virtual void stop() override;
    
    void setModuleName(const std::string& name);

protected:
    virtual Item* doDuplicate() const override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    /* setting parameters */
    double odomUpdateRate;

    /* ROS interfaces */
    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Publisher odomPublisher;
    ros::Subscriber commandSubscriber;
    nav_msgs::Odometry odomMsg;
    Vector3 commandAngularVel;
    Vector3 commandTransVel;

    /* time managers */
    double dt;
    double odomUpdatePeriod;
    double odomLastUpdate;

    /* accessors */
    ControllerIO* io;
    std::ostream& os;
    LinkPtr rootLink;

    /* temporal variables */
    Isometry3 rootLinkPose;
    Vector3 rootLinkAngularVel;
    Vector3 rootLinkTransVel;

    void updateCommandVel(const geometry_msgs::TwistConstPtr msg);
};

typedef ref_ptr<LinkVelocityControllerROSItem> LinkVelocityControllerROSItemPtr;
}  // namespace cnoid

#endif  // CNOID_ROS_PLUGIN_ROOT_LINK_VELOCITY_CONTROL_ITEM_H
