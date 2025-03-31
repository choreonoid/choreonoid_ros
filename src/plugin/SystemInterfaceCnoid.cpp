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


SystemInterfaceCnoid::SystemInterfaceCnoid(cnoid::ControllerIO* io_arg, std::shared_ptr<rclcpp::Node> node_arg)
{
    io = io_arg;
    node = node_arg;
}


CallbackReturn SystemInterfaceCnoid::on_init(const HardwareInfo& info)
{
    auto mv = MessageView::instance();

    // copy the HardwareInfo as `info_ = hardware_info;`
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    const int numJoints = info_.joints.size();

    states.resize(numJoints, {0.0, 0.0, 0.0});
    commands.resize(numJoints, 0);
    controlTypes.resize(numJoints, ControlMode::UNDEFINED);
    gains.resize(numJoints, {0.0, 0.0});

    for (int i = 0; i < info_.joints.size(); ++i) {
        ComponentInfo joint = info_.joints[i];
        const std::string jointName = joint.name;

        // set control types from robot description
        if (joint.command_interfaces.size() != 1) {
            mv->putln(
                formatR(_("joint {} must have one command interface"), jointName),
                MessageView::Error);
            return CallbackReturn::ERROR;
        }

        const std::string controlTypeString = joint.command_interfaces[0].name;
        if (controlTypeString == "position") {
            controlTypes[i] = ControlMode::POSITION;
        } else if (controlTypeString == "velocity") {
            controlTypes[i] = ControlMode::VELOCITY;
        } else if (controlTypeString == "effort") {
            controlTypes[i] = ControlMode::EFFORT;
        } else if (controlTypeString == "position_pd") {
            controlTypes[i] = ControlMode::POSITION_PD;
        } else if (controlTypeString == "velocity_pd") {
            controlTypes[i] = ControlMode::VELOCITY_PD;
        } else {
            mv->putln(
                formatR(_("the command interface of joint {} is invalid"), jointName),
                MessageView::Error);
            return CallbackReturn::ERROR;
        }

        mv->putln(formatR(_("{}:"), jointName));
        mv->putln(formatR(_("\tcontrol type: {}"), controlTypeString));

        // if controlTypes[i] is POSITION, VELOCITY, or EFFORT, skip gain settings
        if (controlTypes[i] <= ControlMode::EFFORT) {
            continue;
        }

        // declare gain parameters
        // parameter names are compatible with the PID Controller in ros2_controllers
        const std::string paramNameP = "gains." + jointName + ".p";
        const std::string paramNameD = "gains." + jointName + ".d";

        auto declareParam = [&](const std::string paramName) {
            try {
                node->declare_parameter<double>(paramName, 0.0);
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
                mv->putln(formatR(_("{}"), e.what()), MessageView::Error);
            } catch (rclcpp::exceptions::InvalidParameterValueException& e) {
                mv->putln(formatR(_("{}"), e.what()), MessageView::Error);
            } catch (rclcpp::exceptions::InvalidParameterTypeException& e) {
                mv->putln(formatR(_("{}"), e.what()), MessageView::Error);
            }
        };

        declareParam(paramNameP);
        declareParam(paramNameD);

        // get gain parameters
        auto getParam = [&](const std::string paramName) -> double {
            try {
                return node->get_parameter(paramName).as_double();
            } catch (rclcpp::exceptions::ParameterNotDeclaredException& e) {
                // this error occurs if the above declaration failed
                // mv->putln(formatR(_("{}"), e.what()), MessageView::Error);
                return 0.0;
            } catch (rclcpp::exceptions::InvalidParameterValueException& e) {
                mv->putln(formatR(_("{}"), e.what()), MessageView::Error);
                return 0.0;
            } catch (rclcpp::exceptions::InvalidParameterTypeException& e) {
                mv->putln(formatR(_("{}"), e.what()), MessageView::Error);
                return 0.0;
            }
        };

        gains[i].p = getParam(paramNameP);
        gains[i].d = getParam(paramNameD);

        mv->putln(formatR(_("\tP gain: {}"), gains[i].p));
        if (gains[i].p <= 0.0) {
            mv->putln(formatR(_("\tP gain should be positive")), MessageView::Warning);
        }
        mv->putln(formatR(_("\tD gain: {}"), gains[i].d));
        if (gains[i].d < 0.0) {
            mv->putln(formatR(_("\tP gain must be non-negative")), MessageView::Error);
        }
    }

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
        switch (controlTypes[i]) {
            case ControlMode::POSITION:
            case ControlMode::POSITION_PD:
                commandInterfaces.emplace_back(CommandInterface(
                    jointName, hardware_interface::HW_IF_POSITION, &commands[i]));
                break;
            case ControlMode::VELOCITY:
            case ControlMode::VELOCITY_PD:
                commandInterfaces.emplace_back(CommandInterface(
                    jointName, hardware_interface::HW_IF_VELOCITY, &commands[i]));
                break;
            case ControlMode::EFFORT:
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
        Link* joint = io->body()->joint(info_.joints[i].name);
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

        switch (controlTypes[i]) {
            case ControlMode::POSITION:
                commands[i] = joint->q();
                joint->setActuationMode(Link::JointDisplacement);
                break;
            case ControlMode::VELOCITY:
                    commands[i] = joint->dq();
                    joint->setActuationMode(Link::JointVelocity);
                    break;
            case ControlMode::EFFORT:
                commands[i] = joint->u();
                joint->setActuationMode(Link::JointEffort);
                break;
            case ControlMode::POSITION_PD:
                commands[i] = joint->q();
                joint->setActuationMode(Link::JointEffort);
                break;
            case ControlMode::VELOCITY_PD:
                commands[i] = joint->dq();
                joint->setActuationMode(Link::JointEffort);
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

        switch (controlTypes[i]) {
            case ControlMode::POSITION:
                joint->q_target() = commands[i];
                break;
            case ControlMode::VELOCITY:
                joint->dq_target() = commands[i];
                break;
            case ControlMode::EFFORT:
                joint->u() = commands[i];
                break;
            case ControlMode::POSITION_PD:
                joint->u() = - gains[i].p * (joint->q() - commands[i]) - gains[i].d * joint->dq();
                break;
            case ControlMode::VELOCITY_PD:
                joint->u() = - gains[i].p * (joint->dq() - commands[i]) - gains[i].d * joint->ddq();
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
