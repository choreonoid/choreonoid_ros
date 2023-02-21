/////////////////////////////////
/// @author Ryodo Tanaka
/////////////////////////////////

#include "RobotHWCnoid.h"
#include <cnoid/MessageView>
#include <cnoid/Body>
#include <cnoid/Link>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <transmission_interface/transmission_parser.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <fmt/format.h>
#include <unordered_map>
#include <limits>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;


namespace hardware_interface {

RobotHWCnoid::RobotHWCnoid()
{
    dof = 0;
}


bool RobotHWCnoid::initSim(const ros::NodeHandle& nh, cnoid::ControllerIO* args)
{
    namespace hi = hardware_interface;

    auto mv = MessageView::instance();

    nodeHandle = nh;
    io = args;

    if(!loadURDF("robot_description")){
        return false;
    }

    dof = transmission.size();
    jointNames.resize(dof);
    controlTypes.resize(dof);
    data.resize(dof);
    command.resize(dof);

    jointTypes.resize(dof);
    limits.resize(dof);

    unordered_map<string, unsigned int> interfaceMap;
    interfaceMap["PositionJointInterface"] = ControlType::POSITION;
    interfaceMap["VelocityJointInterface"] = ControlType::VELOCITY;
    interfaceMap["EffortJointInterface"] = ControlType::EFFORT;
    interfaceMap["hardware_interface/PositionJointInterface"] = ControlType::POSITION;
    interfaceMap["hardware_interface/VelocityJointInterface"] = ControlType::VELOCITY;
    interfaceMap["hardware_interface/EffortJointInterface"] = ControlType::EFFORT;

    // Initialize values
    for(unsigned int i = 0; i < dof; i++){
        // Get joint_interfaces and check them //
        vector<string> jointInterfaces = transmission[i].joints_[0].hardware_interfaces_;
        if(transmission[i].actuators_.empty() && transmission[i].actuators_[0].hardware_interfaces_.empty()){
            mv->putln(
                " should be nested inside the <joint> element, not <actuator>. "
                "The transmission will be properly loaded, but please update "
                "your robot model to remain compatible with future versions of the plugin.",
                MessageView::Warning);
        }
        if(jointInterfaces.empty()){
            mv->putln(
                format(_("{0} on {1} does not specify any hardware interface."),
                       transmission[i].name_, transmission[i].joints_[0].name_),
                MessageView::Warning);
            continue;
        }
        else if(jointInterfaces.size() > 1){
            mv->putln(
                format(_("{0} of transmission specifies multiple hardware interfaces. Currently the default "
                         "robot hardware simulation interface only supports one. Using the first entry"),
                       transmission[i].joints_[0].name_),
                MessageView::Warning);
        }

        // Get joint names //
        jointNames[i] = transmission[i].joints_[0].name_;

        // Get cnoid::Link //
        Link* link = io->body()->joint(jointNames[i].c_str());
        if(!link){
            mv->putln(
                format(_("This robot has a joint named \"{0}\" which is not in the choreonoid model."),
                       jointNames[i]),
                MessageView::Error);
            return false;
        }
        links.emplace_back(link);

        // Store current position, velocity, effort //
        data[i].position = link->q();
        data[i].velocity = link->dq();
        data[i].effort = link->ddq();
        // Set commands by current data //
        command[i].position = data[i].position;
        command[i].velocity = data[i].velocity;
        command[i].effort = data[i].effort;
        
        // Create joint state interface //
        jointStateInterface.registerHandle(
            hi::JointStateHandle(jointNames[i], &data[i].position, &data[i].velocity, &data[i].effort));

        // Set Control type from loaded cnoid::Link or hardware_interface //
        if(link->actuationMode()){
            link->setActuationMode(link->actuationMode());
            switch(link->actuationMode()){
            case Link::JointDisplacement:
                controlTypes[i] = ControlType::POSITION;
                jointHandle = hi::JointHandle(jointStateInterface.getHandle(jointNames[i]), &command[i].position);
                positionJointInterface.registerHandle(jointHandle);
                break;
            case Link::JointVelocity:
                controlTypes[i] = ControlType::VELOCITY;
                jointHandle = hi::JointHandle(jointStateInterface.getHandle(jointNames[i]), &command[i].velocity);
                velocityJointInterface.registerHandle(jointHandle);
                break;
            case Link::JointEffort:
                controlTypes[i] = ControlType::EFFORT;
                jointHandle = hi::JointHandle(jointStateInterface.getHandle(jointNames[i]), &command[i].effort);
                effortJointInterface.registerHandle(jointHandle);
                break;
            default:
                break;
            }
        } else {
            switch(interfaceMap.at(jointInterfaces[0])){
            case ControlType::POSITION:
                controlTypes[i] = ControlType::POSITION;
                jointHandle = hi::JointHandle(jointStateInterface.getHandle(jointNames[i]), &command[i].position);
                positionJointInterface.registerHandle(jointHandle);
                link->setActuationMode(Link::JointDisplacement);
                break;
            case ControlType::VELOCITY:
                controlTypes[i] = ControlType::VELOCITY;
                jointHandle = hi::JointHandle(jointStateInterface.getHandle(jointNames[i]), &command[i].velocity);
                velocityJointInterface.registerHandle(jointHandle);
                link->setActuationMode(Link::JointVelocity);
                break;
            case ControlType::EFFORT:
                controlTypes[i] = ControlType::EFFORT;
                jointHandle = hi::JointHandle(jointStateInterface.getHandle(jointNames[i]), &command[i].effort);
                effortJointInterface.registerHandle(jointHandle);
                link->setActuationMode(Link::JointEffort);
                break;
            default:
                break;
            }
        }
        
        registerJointLimits(i);
    }
    
    // Register interfaces
    registerInterface(&jointStateInterface);
    registerInterface(&positionJointInterface);
    registerInterface(&velocityJointInterface);
    registerInterface(&effortJointInterface);
    
    return true;
}


bool RobotHWCnoid::loadURDF(const std::string& paramName)
{
    auto mv = MessageView::instance();
    string urdfString;

    while(urdfString.empty()){
        string searchedParam;
        if(nodeHandle.searchParam(paramName, searchedParam)){
            if(!nodeHandle.getParam(searchedParam, urdfString)){
                mv->putln(
                    format(_("Waiting for URDF model.\nFinding parameter name : {0}"),
                           searchedParam),
                    MessageView::Warning);
                sleep(1);
            }
        } else {
            mv->putln(format(_("There are no parameter related to : {0}"), paramName),
                      MessageView::Error);
            return false;
        }
    }
    urdfModel = make_shared<urdf::Model>();
    urdfModel->initString(urdfString);

    mv->putln(_("RobotHWCnoid loaded the URDF model."));
    
    // load transmissions //
    transmission_interface::TransmissionParser::parse(urdfString, transmission);
    
    mv->putln(_("RobotHWCnoid loaded Transmission."));
    
    return true;
}


bool RobotHWCnoid::registerJointLimits(const unsigned int& i)
{
    using namespace joint_limits_interface;
    
    auto mv = MessageView::instance();
    
    jointTypes[i] = urdf::Joint::UNKNOWN;
    limits[i].lower.position = -numeric_limits<double>::max();
    limits[i].upper.position = numeric_limits<double>::max();
    limits[i].lower.velocity = 0;
    limits[i].upper.velocity = numeric_limits<double>::max();
    limits[i].lower.effort = 0;
    limits[i].upper.effort = numeric_limits<double>::max();
    JointLimits jointLimits;
    bool hasLimits = false;
    SoftJointLimits softLimits;
    bool hasSoftLimits = false;
    
    if(urdfModel == nullptr){
        mv->putln(_("URDF model is NULL. Abort."), MessageView::Error);
        return false;
    } else {
        const urdf::JointConstSharedPtr urdfJoint = urdfModel->getJoint(jointNames[i]);
        if(!urdfJoint){
            mv->putln(_("URDF model joint is NULL. Abort."), MessageView::Error);
            return false;
        } else {
            jointTypes[i] = urdfJoint->type;
            // Get limits from the URDF file.
            if(getJointLimits(urdfJoint, jointLimits)){
                hasLimits = true;
            }
            if(getSoftJointLimits(urdfJoint, softLimits)){
                hasSoftLimits = true;
            }
        }
    }
    // Limits from parameter //
    if(getJointLimits(jointNames[i], nodeHandle, jointLimits)){
        hasLimits = true;
    }
    // return if there are no limits //
    if(!hasLimits){
        return false;
    }
    // Check joint_type //
    if(jointTypes[i] == urdf::Joint::UNKNOWN){
        if(jointLimits.has_position_limits){
            jointTypes[i] = urdf::Joint::REVOLUTE;
        } else {
            if(jointLimits.angle_wraparound){
                jointTypes[i] = urdf::Joint::CONTINUOUS;
            } else {
                jointTypes[i] = urdf::Joint::PRISMATIC;
            }
        }
    }

    // set limits //
    if(jointLimits.has_position_limits){
        limits[i].lower.position = jointLimits.min_position;
        limits[i].upper.position = jointLimits.max_position;
    }
    if(jointLimits.has_velocity_limits){
        limits[i].upper.velocity = jointLimits.max_velocity;
    }
    if(jointLimits.has_effort_limits){
        limits[i].upper.effort = jointLimits.max_effort;
    }

    // soft limits //
    if(hasSoftLimits){
        switch(controlTypes[i]){
        case ControlType::POSITION:
        {
            const PositionJointSoftLimitsHandle limits_handle(jointHandle, jointLimits, softLimits);
            positionJointSoftLimitInterface_.registerHandle(limits_handle);
            break;
        }
        case ControlType::VELOCITY:
        {
            const VelocityJointSoftLimitsHandle limits_handle(jointHandle, jointLimits, softLimits);
            velocityJointSoftLimitInterface.registerHandle(limits_handle);
            break;
        }
        case ControlType::EFFORT:
        {
            const EffortJointSoftLimitsHandle limits_handle(jointHandle, jointLimits, softLimits);
            effortJointSoftLimitsInterface.registerHandle(limits_handle);
            break;
        }
        default:
            break;
        }
    } else {
        switch (controlTypes[i]){
        case ControlType::POSITION:
        {
            const PositionJointSaturationHandle sat_handle(jointHandle, jointLimits);
            positionJointSaturationInterface.registerHandle(sat_handle);
            break;
        }
        case ControlType::VELOCITY:
        {
            const VelocityJointSaturationHandle sat_handle(jointHandle, jointLimits);
            velocityJointSaturationInterface.registerHandle(sat_handle);
            break;
        }
        case ControlType::EFFORT:
        {
            const EffortJointSaturationHandle sat_handle(jointHandle, jointLimits);
            effortJointSaturationInterface.registerHandle(sat_handle);
            break;
        }
        default:
            break;
        }
    }
    
    return true;
}


void RobotHWCnoid::read(const ros::Time& time, const ros::Duration& period)
{
    for(unsigned int i = 0; i < dof; i++){
        double position = links[i]->q();
        if(jointTypes[i] == urdf::Joint::PRISMATIC){
            data[i].position = position;
        } else {
            data[i].position += angles::shortest_angular_distance(data[i].position, position);
        }
        data[i].velocity = links[i]->dq();
        data[i].effort = links[i]->ddq();
    }
}


void RobotHWCnoid::write(const ros::Time& time, const ros::Duration& period)
{
    positionJointSaturationInterface.enforceLimits(period);
    positionJointSoftLimitInterface_.enforceLimits(period);
    velocityJointSaturationInterface.enforceLimits(period);
    velocityJointSoftLimitInterface.enforceLimits(period);
    effortJointSaturationInterface.enforceLimits(period);
    effortJointSoftLimitsInterface.enforceLimits(period);
    
    for(unsigned int i = 0; i < dof; i++){
        switch (controlTypes[i]){
        case ControlType::POSITION:
            links[i]->q_target() = command[i].position;
            break;
        case ControlType::VELOCITY:
            links[i]->dq_target() = command[i].velocity;
            break;
        case ControlType::EFFORT:
            links[i]->u() = command[i].effort;
            break;
        default:
            break;
        }
    }
}

}  // namespace hardware_interface


PLUGINLIB_EXPORT_CLASS(hardware_interface::RobotHWCnoid, hardware_interface::RobotHWSim<cnoid::ControllerIO*>)
