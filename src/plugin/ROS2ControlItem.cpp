#include "ROS2ControlItem.h"

#include <cnoid/Archive>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/PutPropertyFunction>
#include "Format.h"
#include "gettext.h"

#include <controller_manager/controller_manager.hpp>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

using namespace std;
using namespace cnoid;
using fmt::format;


void ROS2ControlItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager()
        .registerClass<ROS2ControlItem>(N_("ROS2ControlItem"))
        .addCreationPanel<ROS2ControlItem>();
}


ROS2ControlItem::ROS2ControlItem()
{

}


ROS2ControlItem::ROS2ControlItem(const ROS2ControlItem& org)
    : ControllerItem(org)
{

}


ROS2ControlItem::~ROS2ControlItem()
{
    if (controllerManager) {
        executor->remove_node(controllerManager);
        executor->cancel();
    }
}


Item* ROS2ControlItem::doDuplicate() const
{
    return new ROS2ControlItem(*this);
}


bool ROS2ControlItem::store(Archive& archive)
{
    if(!nodeNamespace.empty()){
        archive.write("namespace", nodeNamespace);
    }
    return true;
}

bool ROS2ControlItem::restore(const Archive& archive)
{
    if(!archive.read({"namespace"}, nodeNamespace)){
        nodeNamespace.clear();
    }
    return true;
}


void ROS2ControlItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Namespace"), nodeNamespace, changeProperty(nodeNamespace));
}


bool ROS2ControlItem::initialize(ControllerIO* io)
{
    auto mv = MessageView::instance();
    
    // check if Choreonoid is executed as a ROS 2 node
    if (!rclcpp::ok()) {
        mv->putln(
            formatR(_("Choreonoid is not executed as a ROS 2 node")),
            MessageView::Error);
    }

    // check the body
    if (!io->body()){
        mv->putln(
            formatR(_("ROS2ControlItem \"{}\" is invalid because it is not assigned to a body"),
                    displayName()),
            MessageView::Error);
        return false;
    }

    // copy controller interface into the private member
    this->io = io;

    if (!nodeNamespace.empty()) {
        robotStatePublisherName = nodeNamespace + "/" + robotStatePublisherName;
    }


    // create ros2_control node
    const std::string nodeName = "choreonoid_ros2_control";
    node = rclcpp::Node::make_shared(nodeName, nodeNamespace);
    
    // create an executor and a executor thread
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executorThread = std::thread([this](){ executor->spin(); });

    // get urdf from the robot state publisher
    const std::string urdfString = getURDF();

    // construct hardwareInfo from the URDF data
    std::vector<hardware_interface::HardwareInfo> controlHardwareInfo;
    try {
        controlHardwareInfo = hardware_interface::parse_control_resources_from_urdf(urdfString);
    } catch (const std::runtime_error& error) {
        mv->putln(
            formatR(_("Failed to parse the robot URDF: {}"), error.what()),
            MessageView::Error);
        return false;
    }

    // initialize ResourceManager
    std::unique_ptr<hardware_interface::ResourceManager> resourceManager = std::make_unique<hardware_interface::ResourceManager>();
    try {
        resourceManager->load_urdf(urdfString, false, false);
    } catch (...) {
        mv->putln(formatR(_("Failed to initialize ResourceManager")), MessageView::Error);
    }

    // initialize SystemInterfaceCnoid
    std::unique_ptr<SystemInterfaceCnoid> interface(new SystemInterfaceCnoid(io, node));
    resourceManager->import_component(std::move(interface), controlHardwareInfo[0]);
    
    // activate the corresponding HardwareComponent
    rclcpp_lifecycle::State state(
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
        hardware_interface::lifecycle_state_names::ACTIVE);
    resourceManager->set_component_state(controlHardwareInfo[0].name, state);

    // create controller manager
    mv->putln(formatR(_("Creating ControllerManager")));
    const std::string controllerManagerName = "controller_manager";
    controllerManager.reset(
        new controller_manager::ControllerManager(
            std::move(resourceManager),
            executor,
            controllerManagerName,
            nodeNamespace));
    executor->add_node(controllerManager);

    const int updateRate = controllerManager->get_parameter("update_rate").as_int();
    if (updateRate < 0.1) {
        mv->putln(
            formatR(_("ROS2ControlItem {} gets an invalid update rate: {}. It should be >= 0.1"),
                    displayName(), updateRate),
            MessageView::Error);
    }
    controlPeriod.reset(new rclcpp::Duration(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / static_cast<double>(updateRate)))));

    // set "use_sim_time" parameter to the controller manager
    controllerManager->set_parameter(
        rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

    mv->putln(formatR(_("{} has successfully been initialized"), displayName()));

    return true;
}


bool ROS2ControlItem::start()
{
    // set time step
    const double timeStep = io->timeStep();
    const int timeStepSec = static_cast<int>(timeStep);
    const int timeStepNsec = static_cast<int>(timeStep * 1e9);

    period.reset(new rclcpp::Duration(timeStepSec, timeStepNsec));

    return true;
}


void ROS2ControlItem::input()
{
    // get current time
    const double currentTime = io->currentTime();
    const int nowSec = static_cast<int>(currentTime);
    const int nowNsec = static_cast<int>((currentTime - nowSec) * 1e9);

    // RCL_ROS_TIME parameter is essential to use simulation time
    now = rclcpp::Time(nowSec, nowNsec, RCL_ROS_TIME);

    // read joint states
    controllerManager->read(now, *period);
}


bool ROS2ControlItem::control()
{  
    // update control commands
    // TODO: apply update_rate param of controllerManager
    controllerManager->update(now, *period);

    return true;
}


void ROS2ControlItem::output()
{
    // write commands into joints
    controllerManager->write(now, *period);
}


void ROS2ControlItem::stop()
{

}


std::string ROS2ControlItem::getURDF() const
{
    std::string urdfString;
    auto mv = MessageView::instance();

    using namespace std::chrono_literals;
    auto client = std::make_shared<rclcpp::AsyncParametersClient>(
        node, robotStatePublisherName);

    mv->putln(formatR(_("Try to connect to {} ..."), robotStatePublisherName.data()));

    while (!client->wait_for_service(0.1s)) {}

    mv->putln(formatR(_("Connected to {}"), robotStatePublisherName.data()));

    // search and wait for robot_description parameter
    mv->putln(
        formatR(_("Try to get URDF parameter {} from {} ..."),
                robotDescription.data(),
                robotStatePublisherName.data()));

    try {
        auto f = client->get_parameters({robotDescription});
        f.wait();
        std::vector<rclcpp::Parameter> values = f.get();
        urdfString = values[0].as_string();
    } catch (const std::exception& error) {
        mv->putln(
            formatR(_("Failed to get the URDF parameter: {}"), error.what()),
            MessageView::Error);
    }

    if (urdfString.empty()) {
        mv->putln(
            formatR(_("Parameter {} of {} is empty"),
                    robotDescription.data(),
                    robotStatePublisherName.data()),
            MessageView::Error);
    }

    mv->putln(formatR(_("Received URDF parameter from {}"), robotStatePublisherName.data()));

    return urdfString;
}
