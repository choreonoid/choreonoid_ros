///////////////////////////////////////////////////////////
/// @file robot_hw_sim.h
/// @brief ros_control interface for Simulator
/// @author Ryodo Tanaka 
///////////////////////////////////////////////////////////

#ifndef HARDWARE_INTERFACE_ROBOT_HW_SIM_H
#define HARDWARE_INTERFACE_ROBOT_HW_SIM_H

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>

namespace hardware_interface
{
template<typename... Ts>
class RobotHWSim : public RobotHW
{
 public:
  RobotHWSim(void) = default;
  ~RobotHWSim(void) =  default;

  virtual bool initSim(const ros::NodeHandle& nh, Ts... args) = 0;
  virtual void read(const ros::Time& time, const ros::Duration& period) override;
  virtual void write(const ros::Time& time, const ros::Duration& period) override;
};

} // namespace hardware_interface

#endif // HARDWARE_INTERFACE_ROBOT_HW_SIM_H
