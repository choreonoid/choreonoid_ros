///////////////////////////////////////////////////////////
/// @file RobotHWCnoid.h
/// @brief ros_control interface for Choreonoid
/// @author Ryodo Tanaka
///////////////////////////////////////////////////////////

#ifndef HARDWARE_INTERFACE_ROBOT_HW_CNOID_H
#define HARDWARE_INTERFACE_ROBOT_HW_CNOID_H

#include "RobotHWSim.h"
#include <cnoid/ControllerIO>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <transmission_interface/transmission_info.h>
#include <urdf/model.h>
#include <vector>

namespace hardware_interface {

class RobotHWCnoid : public RobotHWSim<cnoid::ControllerIO*>
{
public:
    RobotHWCnoid();
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
    ros::NodeHandle nodeHandle;
    cnoid::ControllerIO* io;

    // transmissions //
    std::vector<transmission_interface::TransmissionInfo> transmission;

    // urdf model //
    std::shared_ptr<urdf::Model> urdfModel;

    // Joint Handle //
    hardware_interface::JointHandle jointHandle;

    // Interface //
    hardware_interface::JointStateInterface jointStateInterface;
    hardware_interface::PositionJointInterface positionJointInterface;
    hardware_interface::VelocityJointInterface velocityJointInterface;
    hardware_interface::EffortJointInterface effortJointInterface;
    
    // cnoid::Link //
    std::vector<cnoid::Link*> links;
    
    // Joint limits //
    std::vector<Limits<Data>> limits;
    std::vector<int> jointTypes;
    joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;
    joint_limits_interface::PositionJointSoftLimitsInterface positionJointSoftLimitInterface_;
    joint_limits_interface::VelocityJointSaturationInterface velocityJointSaturationInterface;
    joint_limits_interface::VelocityJointSoftLimitsInterface velocityJointSoftLimitInterface;
    joint_limits_interface::EffortJointSaturationInterface effortJointSaturationInterface;
    joint_limits_interface::EffortJointSoftLimitsInterface effortJointSoftLimitsInterface;
    
    unsigned int dof;
    std::vector<std::string> jointNames;
    std::vector<ControlType> controlTypes;
    std::vector<Data> data;
    std::vector<Data> command;
};

typedef std::shared_ptr<RobotHWCnoid> RobotHWCnoidPtr;

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE_ROBOT_HW_CNOID_H
