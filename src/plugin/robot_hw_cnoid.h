///////////////////////////////////////////////////////////
/// @file robot_hw_cnoid.h
/// @brief ros_control interface for Choreonoid
/// @author Ryodo Tanaka
///////////////////////////////////////////////////////////

#ifndef HARDWARE_INTERFACE_ROBOT_HW_CNOID_H
#define HARDWARE_INTERFACE_ROBOT_HW_CNOID_H

// ROS //
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/ros.h>
#include <transmission_interface/transmission_info.h>
#include <urdf/model.h>

#include "robot_hw_sim.h"

// STL //
#include <unordered_map>
#include <vector>

// Cnoid //
#include <cnoid/ControllerIO>

namespace hardware_interface
{
class RobotHWCnoid : public RobotHWSim<cnoid::ControllerIO*>
{
public:
  virtual bool initSim(const ros::NodeHandle& nh, cnoid::ControllerIO* args) final;
  virtual void read(const ros::Time& time, const ros::Duration& period) final;
  virtual void write(const ros::Time& time, const ros::Duration& period) final;

private:
  enum ControlType
  {
    POSITION,
    VELOCITY,
    EFFORT,
    POSITION_PID,
    VELOCITY_PID,
    EFFORT_PID
  };
  template <typename T>
  struct Limits
  {
    T lower;
    T upper;
  };
  struct Data
  {
    double position;
    double velocity;
    double effort;
  };

  bool loadURDF(const std::string& param_name = "robot_description");
  bool registerJointLimits(const unsigned int& i);

  // arguments //
  ros::NodeHandle nh_;
  cnoid::ControllerIO* io_;

  // transmissions //
  std::vector<transmission_interface::TransmissionInfo> tmss_;

  // urdf model //
  std::shared_ptr<urdf::Model> urdf_model_;

  // Joint Handle //
  hardware_interface::JointHandle joint_handle_;

  // Interface //
  hardware_interface::JointStateInterface js_if_;
  hardware_interface::PositionJointInterface pj_if_;
  hardware_interface::VelocityJointInterface vj_if_;
  hardware_interface::EffortJointInterface ej_if_;

  // cnoid::Link //
  std::vector<cnoid::Link*> links_;

  // Joint limits //
  std::vector<Limits<Data>> limits_;
  std::vector<int> joint_types_;
  joint_limits_interface::PositionJointSaturationInterface pj_sat_if_;
  joint_limits_interface::PositionJointSoftLimitsInterface pj_lim_if_;
  joint_limits_interface::VelocityJointSaturationInterface vj_sat_if_;
  joint_limits_interface::VelocityJointSoftLimitsInterface vj_lim_if_;
  joint_limits_interface::EffortJointSaturationInterface ej_sat_if_;
  joint_limits_interface::EffortJointSoftLimitsInterface ej_lim_if_;

  unsigned int dof_{ 0 };
  std::vector<std::string> joint_names_;
  std::vector<ControlType> ctrl_types_;
  std::vector<Data> data_;
  std::vector<Data> command_;
  std::unordered_map<std::string, int> joint_map_;
};

typedef std::shared_ptr<RobotHWCnoid> RobotHWCnoidPtr;

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE_ROBOT_HW_CNOID_H
