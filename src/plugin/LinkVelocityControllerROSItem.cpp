/**
   \file
   \author Yuki Onishi
*/

#include "LinkVelocityControllerROSItem.h"
#include <cnoid/Body>
#include <cnoid/BodyItem>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>

#include <geometry_msgs/Twist.h>

#include <algorithm>
#include <string>

#include <fmt/format.h>
#include "gettext.h"

using namespace cnoid;
using fmt::format;

void LinkVelocityControllerROSItem::initializeClass(ExtensionManager* ext)
{ 
    ext->itemManager().registerClass<LinkVelocityControllerROSItem>(N_("LinkVelocityControllerROSItem"));
    ext->itemManager().addCreationPanel<LinkVelocityControllerROSItem>();
}


LinkVelocityControllerROSItem::LinkVelocityControllerROSItem()
    : os(MessageView::instance()->cout())
{
    io = nullptr;
    odomUpdateRate = 50.0;
}


LinkVelocityControllerROSItem::LinkVelocityControllerROSItem(const LinkVelocityControllerROSItem& org)
    : ControllerItem(org)
    , os(MessageView::instance()->cout())
{
    io = nullptr;
    odomUpdateRate = 50.0;
}


LinkVelocityControllerROSItem::~LinkVelocityControllerROSItem()
{
    stop();
}


Item* LinkVelocityControllerROSItem::doDuplicate() const
{
    return new LinkVelocityControllerROSItem(*this);
}


bool LinkVelocityControllerROSItem::store(Archive& archive)
{
    archive.write("odometry_update_rate", odomUpdateRate);

    return true;
}


bool LinkVelocityControllerROSItem::restore(const Archive& archive)
{
    archive.read("odometry_update_rate", odomUpdateRate);
    return true;
}


void LinkVelocityControllerROSItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty.decimals(2).min(0.0)("Odometry update rate", odomUpdateRate, changeProperty(odomUpdateRate));
}


bool LinkVelocityControllerROSItem::initialize(ControllerIO* io)
{
    if (!io->body()) {
        MessageView::instance()->putln(
            format(_("LinkVelocityControllerROSItem \"{0}\" is invalid because it is not assigned to a body."), displayName()),
            MessageView::WARNING);
        return false;
    }

    this->io = io;
    dt = io->timeStep();

    return true;
}


bool LinkVelocityControllerROSItem::start()
{
    // gets target link ptr
    rootLink = io->body()->rootLink();
    if (!rootLink) {
        ROS_ERROR("Failed to get root link");
        return false;
    }
    // sets target link properties
    rootLink->setActuationMode(Link::LinkPosition);

    // initializes the Odom msg to publish
    odomMsg.header.frame_id = "odom";
    std::string rootLinkName = rootLink->name();
    std::replace(rootLinkName.begin(), rootLinkName.end(), '-', '_');
    odomMsg.child_frame_id = rootLinkName;

    // initializes a publisher and a subscriber
    std::string name = io->body()->name();
    std::replace(name.begin(), name.end(), '-', '_');
    rosNode.reset(new ros::NodeHandle(name));

    odomPublisher =
        rosNode->advertise<nav_msgs::Odometry>("odom", 1);
    commandSubscriber = rosNode->subscribe("cmd_vel", 1,
        &LinkVelocityControllerROSItem::updateCommandVel, this);
    commandAngularVel = Vector3::Zero();
    commandTransVel = Vector3::Zero();

    // calculates time parameters
    odomUpdatePeriod = 1.0 / odomUpdateRate;
    odomLastUpdate = io->currentTime();

    ROS_DEBUG("Odometry update rate %f", odomUpdateRate);

    return true;
}


void LinkVelocityControllerROSItem::input()
{
    rootLinkPose = rootLink->position();
    rootLinkTransVel = rootLink->v();
    rootLinkAngularVel = rootLink->w();
}


bool LinkVelocityControllerROSItem::control()
{
    const double now = io->currentTime();
    const double elapsedTime = now - odomLastUpdate;

    // updates & publishes odometry
    if (elapsedTime > odomUpdatePeriod) {
        odomMsg.header.stamp.fromSec(now);

        const Vector3& p = rootLinkPose.translation();
        odomMsg.pose.pose.position.x = p.x();
        odomMsg.pose.pose.position.y = p.y();
        odomMsg.pose.pose.position.z = p.z();
        const Eigen::Quaterniond q =
            Eigen::Quaterniond(rootLinkPose.rotation());
        odomMsg.pose.pose.orientation.w = q.w();
        odomMsg.pose.pose.orientation.x = q.x();
        odomMsg.pose.pose.orientation.y = q.y();
        odomMsg.pose.pose.orientation.z = q.z();

        odomMsg.twist.twist.linear.x = rootLinkTransVel.x();
        odomMsg.twist.twist.linear.y = rootLinkTransVel.y();
        odomMsg.twist.twist.linear.z = rootLinkTransVel.z();
        odomMsg.twist.twist.angular.x = rootLinkAngularVel.x();
        odomMsg.twist.twist.angular.y = rootLinkAngularVel.y();
        odomMsg.twist.twist.angular.z = rootLinkAngularVel.z();

        odomPublisher.publish(odomMsg);
        odomLastUpdate += odomUpdatePeriod;
    }

    return true;
}


void LinkVelocityControllerROSItem::output()
{
    // moves root link by Eular's integration of command velocity
    rootLink->translation() += rootLinkPose.rotation() * commandTransVel * dt;

    const Vector3 w = commandAngularVel * dt;
    const double theta = w.norm();
    const Vector3 a = w.normalized();
    Matrix3 ah;
    ah << 0.0, -a.z(), a.y(), a.z(), 0.0, -a.x(), -a.y(), a.x(), 0.0;
    const Matrix3 dR =
        Matrix3::Identity() + ah * sin(theta) + ah * ah * (1.0 - cos(theta));
    rootLink->rotation() = rootLinkPose.rotation() * dR;
}


void LinkVelocityControllerROSItem::stop()
{
    if (ros::ok()) {
        odomPublisher.shutdown();
        commandSubscriber.shutdown();

        if (rosNode) {
            rosNode->shutdown();
        }
    }
}

void LinkVelocityControllerROSItem::updateCommandVel(
    const geometry_msgs::TwistConstPtr msg) {
        commandTransVel =
            Vector3(msg->linear.x, msg->linear.y, msg->linear.z);
        commandAngularVel =
            Vector3(msg->angular.x, msg->angular.y, msg->angular.z);
}
