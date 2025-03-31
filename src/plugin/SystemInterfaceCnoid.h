#ifndef SYSTEM_INTERFACE_CNOID_H
#define SYSTEM_INTERFACE_CNOID_H

#include <cnoid/ControllerIO>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>

using hardware_interface::CallbackReturn;
using hardware_interface::CommandInterface;
using hardware_interface::ComponentInfo;
using hardware_interface::HardwareInfo;
using hardware_interface::return_type;
using hardware_interface::StateInterface;
using hardware_interface::SystemInterface;

namespace cnoid {

class SystemInterfaceCnoid : public SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SystemInterfaceCnoid);

  SystemInterfaceCnoid();

  explicit SystemInterfaceCnoid(cnoid::ControllerIO* io, std::shared_ptr<rclcpp::Node> node);

  CallbackReturn on_init(const HardwareInfo& info) override;

  CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;

  std::vector<StateInterface> export_state_interfaces() override;

  std::vector<CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  return_type read(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

  return_type write(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    template <typename T>
    struct Limits
    {
        T lower;
        T upper;
    };
    struct State
    {
        double position;
        double velocity;
        double effort;
    };
    struct Gain
    {
        double p;
        double d;
    };

    enum ControlMode {
        POSITION,
        VELOCITY,
        EFFORT,
        POSITION_PD,
        VELOCITY_PD,
        UNDEFINED
    };



    cnoid::ControllerIO* io;

    std::shared_ptr<rclcpp::Node> node = nullptr;

    std::vector<State> states;
    std::vector<double> commands;
    std::vector<ControlMode> controlTypes;
    std::vector<Gain> gains;
};

}  // namespace cnoid

#endif  // SYSTEM_INTERFACE_CNOID_H
