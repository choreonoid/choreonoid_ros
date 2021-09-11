#ifndef CNOID_DIFF_DRIVE_CONTROLLER_ITEM_H
#define CNOID_DIFF_DRIVE_CONTROLLER_ITEM_H

#include <cnoid/Archive>
#include <cnoid/Body>
#include <cnoid/BodyItem>
#include <cnoid/ControllerItem>
#include <cnoid/ItemManager>
#include <cnoid/Link>
#include <cnoid/MessageView>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include "exportdecl.h"

namespace cnoid
{
class CNOID_EXPORT DiffDriveControllerItem : public ControllerItem
{
public:
  static void initializeClass(ExtensionManager * ext);

  DiffDriveControllerItem();
  DiffDriveControllerItem(const DiffDriveControllerItem & org);
  virtual ~DiffDriveControllerItem();

  virtual bool initialize(ControllerIO* io) override;
  virtual bool start() override;

  virtual double timeStep() const override { return timeStep_; }
  virtual void input() override;
  virtual bool control() override;
  virtual void output() override;
  virtual void stop() override;

  void callbackCmdVel(const geometry_msgs::Twist & msg);
  void calcOdom(double vel_l, double vel_r);

  const BodyPtr & body() const { return simulationBody_; }
  double controlTime() const { return controlTime_; }
protected:
  virtual Item * doDuplicate() const override;
  virtual bool store(Archive & archive) override;
  virtual bool restore(const Archive & archive) override;
  void doPutProperties(PutPropertyFunction & putProperty) override;

  BodyPtr simulationBody_;
  double timeStep_;

  ControllerIO* io;
  const Target * controllerTarget;
  double controlTime_;

  // 目標並進速度
  double target_v_;
  // 目標旋回速度
  double target_w_;
  // 車輪中心軸から車輪までの距離
  double wheel_base_;
  // 車輪の半径
  double wheel_radius_;
  Link * wheel_[2];

  double prev_odom_[3];

  std::string bodyName;

  ros::Time prev_time_, curr_time_;
  ros::Subscriber cmd_vel_sub_;
  ros::Publisher odom_pub_;
  boost::shared_ptr<ros::NodeHandle> ros_node_;
  boost::shared_ptr<ros::AsyncSpinner> async_ros_spin_;
};

typedef ref_ptr<DiffDriveControllerItem> DiffDriveControllerItemPtr;
}

#endif
