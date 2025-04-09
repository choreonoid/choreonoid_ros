#include "SystemInterfaceCnoid.h"

#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/MessageView>
#include "Format.h"
#include "gettext.h"

#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace cnoid {

SystemInterfaceCnoid::SystemInterfaceCnoid(){
}


SystemInterfaceCnoid::SystemInterfaceCnoid(cnoid::ControllerIO* io_arg)
{
    io = io_arg;
}


CallbackReturn SystemInterfaceCnoid::on_init(const HardwareInfo& info)
{
    // copy the HardwareInfo as `info_ = hardware_info;`
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }
 
    states.resize(info_.joints.size(), { 0.0, 0.0, 0.0 });
    commands.resize(info_.joints.size(), 0);

    return CallbackReturn::SUCCESS;
}


CallbackReturn SystemInterfaceCnoid::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
    return CallbackReturn::SUCCESS;
}


std::vector<StateInterface> SystemInterfaceCnoid::export_state_interfaces()
{
    std::vector<StateInterface> stateInterfaces;
    stateInterfaces.reserve(3 * info_.joints.size());
    for (int i = 0; i < info_.joints.size(); ++i) {
        const std::string jointName = info_.joints[i].name;
        stateInterfaces.emplace_back(StateInterface(
        jointName, hardware_interface::HW_IF_POSITION, &states[i].position));
        stateInterfaces.emplace_back(StateInterface(
        jointName, hardware_interface::HW_IF_VELOCITY, &states[i].velocity));
        stateInterfaces.emplace_back(StateInterface(
        jointName, hardware_interface::HW_IF_EFFORT, &states[i].effort));
    }

    return stateInterfaces;
}


std::vector<CommandInterface> SystemInterfaceCnoid::export_command_interfaces()
{
    std::vector<CommandInterface> commandInterfaces;
    commandInterfaces.reserve(info_.joints.size());
    for (int i = 0; i < info_.joints.size(); ++i) {
        const std::string jointName = info_.joints[i].name;
        Link* joint = io->body()->joint(jointName);
        if (!joint) {
            auto mv = MessageView::instance();
            mv->putln(
                formatR(_("joint {} is not found in the simulation body"), info_.joints[i].name),
                MessageView::Error);
            continue;
        }

        switch (joint->actuationMode()) {
            case Link::JointDisplacement:
                commandInterfaces.emplace_back(CommandInterface(
                    jointName, hardware_interface::HW_IF_POSITION, &commands[i]));
                break;
            case Link::JointVelocity:
                commandInterfaces.emplace_back(CommandInterface(
                    jointName, hardware_interface::HW_IF_VELOCITY, &commands[i]));
                break;
            case Link::JointEffort:
                commandInterfaces.emplace_back(CommandInterface(
                    jointName, hardware_interface::HW_IF_EFFORT, &commands[i]));
                break;
            default:
                // TODO: error! set actuation_mode of joint {0} in the body description file
                break;
        }
    }

    return commandInterfaces;
}


CallbackReturn SystemInterfaceCnoid::on_activate(
    const rclcpp_lifecycle::State& previous_state)
{
    for (int i = 0; i < info_.joints.size(); ++i) {
        const Link* joint = io->body()->joint(info_.joints[i].name);
        if (!joint) {
            auto mv = MessageView::instance();
            mv->putln(
                formatR(_("joint {} is not found in the simulation body"), info_.joints[i].name),
                MessageView::Error);
            return CallbackReturn::ERROR;
        }

        states[i].position = joint->q();
        states[i].velocity = joint->dq();
        states[i].effort = joint->u();

        switch (joint->actuationMode()) {
            case Link::JointDisplacement:
                commands[i] = joint->q();
                break;
            case Link::JointVelocity:
                commands[i] = joint->dq();
                break;
            case Link::JointEffort:
                commands[i] = joint->u();
                break;
            default:
                // TODO: output error message
                return CallbackReturn::ERROR;
        }
    }

    return CallbackReturn::SUCCESS;
}


CallbackReturn SystemInterfaceCnoid::on_deactivate(
    const rclcpp_lifecycle::State& previous_state)
{
    return CallbackReturn::SUCCESS;
}


return_type SystemInterfaceCnoid::read(
    const rclcpp::Time& time, const rclcpp::Duration& period)
{
    // copy joint states from the simulation body
    for (int i = 0; i < info_.joints.size(); ++i) {
        const Link* joint = io->body()->joint(info_.joints[i].name);
        states[i].position = joint->q();
        states[i].velocity = joint->dq();
        states[i].effort = joint->u();
    }

    return return_type::OK;
}


return_type SystemInterfaceCnoid::write(
    const rclcpp::Time& time, const rclcpp::Duration& period)
{
    // TODO: enforces joint limits

    // copy control commands to the simulation body
    for (int i = 0; i < info_.joints.size(); ++i) {
        Link* joint = io->body()->joint(info_.joints[i].name);
        switch (joint->actuationMode()) {
            case Link::JointDisplacement:
                joint->q_target() = commands[i];
                break;
            case Link::JointVelocity:
                joint->dq_target() = commands[i];
                break;
            case Link::JointEffort:
                joint->u() = commands[i];
                break;
            default:
                // TODO: output error message
                return return_type::ERROR;
        }
    }

    return return_type::OK;
}

}  // namespace cnoid

PLUGINLIB_EXPORT_CLASS(cnoid::SystemInterfaceCnoid, hardware_interface::SystemInterface)
