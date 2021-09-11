#include "DiffDriveControllerItem.h"
#include "gettext.h"
#include <fmt/format.h>

using namespace cnoid;
using fmt::format;

void DiffDriveControllerItem::initializeClass(ExtensionManager* ext)
{
  ext->itemManager().registerClass<DiffDriveControllerItem>(N_("DiffDriveControllerItem"));
  ext->itemManager().addCreationPanel<DiffDriveControllerItem>();
}

DiffDriveControllerItem::DiffDriveControllerItem()
: wheel_base_(1.2), wheel_radius_(0.2), target_v_(0.0), target_w_(0.0)
{
  prev_odom_[0] = 0.0;
  prev_odom_[1] = 0.0;
  prev_odom_[2] = 0.0;

  io = nullptr;
}

DiffDriveControllerItem::DiffDriveControllerItem(const DiffDriveControllerItem & org)
: ControllerItem(org), wheel_base_(1.2), wheel_radius_(0.2), target_v_(0.0), target_w_(0.0)
{
  prev_odom_[0] = 0.0;
  prev_odom_[1] = 0.0;
  prev_odom_[2] = 0.0;

  io = nullptr;
}

DiffDriveControllerItem::~DiffDriveControllerItem()
{
  stop();
}

Item* DiffDriveControllerItem::doDuplicate() const
{
  std::cout << "doDuplicate" << std::endl;
  return new DiffDriveControllerItem(*this);
  std::cout << "finish doDuplicate" << std::endl;
}
void DiffDriveControllerItem::doPutProperties(PutPropertyFunction& putProperty)
{
  return;
}
bool DiffDriveControllerItem::store(Archive& archive)
{
  return true;
}
bool DiffDriveControllerItem::restore(const Archive& archive)
{
  return true;
}

bool DiffDriveControllerItem::initialize(ControllerIO* io)
{
  if (!io->body()) {
      MessageView::instance()->putln(
          format(_("DiffDriveControllerItem \"{0}\" is invalid because it is not assigned to a body."), displayName()),
          MessageView::WARNING);
      return false;
  }

  this->io = io;
  simulationBody_ = io->body();
  timeStep_ = io->worldTimeStep();
  controlTime_ = io->currentTime();

  return true;
}
bool DiffDriveControllerItem::start()
{
  std::vector<std::string> wheel_name = { "TRACK_L", "TRACK_R" };
  for (size_t i = 0; i < wheel_name.size(); i++) {
    wheel_[i] = simulationBody_->link(wheel_name[i].c_str());
    wheel_[i]->setActuationMode(wheel_[i]->actuationMode());
  }

  std::string name = body()->name();
  std::replace(name.begin(), name.end(), '-', '_');

  ros_node_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle(name));

  odom_pub_ = ros_node_->advertise<nav_msgs::Odometry>("/DoubleArm/odom", 10);
  cmd_vel_sub_ = ros_node_->subscribe("/DoubleArm/cmd_vel", 10, &DiffDriveControllerItem::callbackCmdVel, this);

  async_ros_spin_.reset(new ros::AsyncSpinner(0));
  async_ros_spin_->start();

  prev_time_ = ros::Time::now();

  return true;
}

bool DiffDriveControllerItem::control()
{
  double wheel_vel[2];
  wheel_vel[0] = target_v_ - (target_w_ * (wheel_base_ / 2.0));
  wheel_vel[1] = target_v_ + (target_w_ * (wheel_base_ / 2.0));

  // 車輪の目標速度
  for (std::size_t i = 0; i < 2; i++)
    wheel_[i]->dq_target() = wheel_vel[i] / wheel_radius_;

  return true;
}

void DiffDriveControllerItem::input()
{
  return;
}
void DiffDriveControllerItem::output()
{
  return;
}
void DiffDriveControllerItem::stop()
{
  return;
}
void DiffDriveControllerItem::callbackCmdVel(const geometry_msgs::Twist& msg)
{
  target_v_ = msg.linear.x;
  target_w_ = msg.angular.z;
}
