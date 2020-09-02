///////////////////////////////////////////////////////////
/// @file RobotHWSim.h
/// @brief RobotHW interface for Simulation
/// @author Ryodo Tanaka 
///////////////////////////////////////////////////////////

#ifndef HARDWARE_INTERFACE_CNOID_HW_H
#define HARDWARE_INTERFACE_CNOID_HW_H

// ROS //
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <transmission_interface/transmission_info.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <urdf/model.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

namespace hardware_interface
{
class CnoidHW : public RobotHW
{
 public:
  virtual bool init(ros::NodeHandle& robot_nh, ros::NodeHandle& robot_hw_nh) override;
  
  virtual void read(const ros::Time& time, const ros::Duration& period) override;
  virtual void write(const ros::Time& time, const ros::Duration& period) override;

 private:

  bool loadURDF(const std::string& param_name="robot_description");

  ros::NodeHandle nh_;
  std::vector<transmission_interface::TransmissionInfo> tmss_;
  urdf::Model urdf_model_;
  std::string robot_description_;
  
  uint dof_{0};
  hardware_interface::JointStateInterface    js_if_;
  hardware_interface::PositionJointInterface pj_if_;
  hardware_interface::VelocityJointInterface vj_if_;
  hardware_interface::EffortJointInterface   ej_if_;

  joint_limits_interface::PositionJointSaturationInterface pj_sat_if_;
  joint_limits_interface::PositionJointSoftLimitsInterface pj_lim_if_;
  joint_limits_interface::VelocityJointSaturationInterface vj_sat_if_;
  joint_limits_interface::VelocityJointSoftLimitsInterface vj_lim_if_;
  joint_limits_interface::EffortJointSaturationInterface   ej_sat_if_;
  joint_limits_interface::EffortJointSoftLimitsInterface   ej_lim_if_;
};

typedef std::shared_ptr<CnoidHW> CnoidHWPtr;
} // namespace hardware_interface

#endif // HARDWARE_INTERFACE_CNOID_HW_H
