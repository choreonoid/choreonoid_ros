/////////////////////////////////
/// @author Ryodo Tanaka
/////////////////////////////////

#include "ROSControlItem.h"
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <cnoid/PutPropertyFunction>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using namespace hardware_interface;
using fmt::format;


void ROSControlItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager()
        .registerClass<ROSControlItem>(N_("ROSControlItem"))
        .addCreationPanel<ROSControlItem>();
}


ROSControlItem::ROSControlItem()
{
    io = nullptr;
}


ROSControlItem::ROSControlItem(const ROSControlItem& org)
    : ControllerItem(org)
{
    io = nullptr;
}


ROSControlItem::~ROSControlItem()
{
    // stop();
}


Item* ROSControlItem::doDuplicate() const
{
    return new ROSControlItem(*this);
}


bool ROSControlItem::store(Archive& archive)
{
    if(!nodeNamespace.empty()){
        archive.write("name_space", nodeNamespace);
    }
    return true;
}

bool ROSControlItem::restore(const Archive& archive)
{
    if(!archive.read({ "name_space", "name space" }, nodeNamespace)){
        nodeNamespace.clear();
    }
    return true;
}


void ROSControlItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Name space"), nodeNamespace, changeProperty(nodeNamespace));
}


bool ROSControlItem::initialize(ControllerIO* io)
{
    auto mv = MessageView::instance();
    
    // Check body //
    if(!io->body()){
        mv->putln(
            format(_("ROSControlItem \"{0}\" is invalid because it is not assigned to a body."),
                   displayName()),
            MessageView::Error);
        return false;
    }

    // Check that ROS has been initialized
    if(!ros::isInitialized()){
        mv->putln("ROS master is not initialized.", MessageView::Warning);
        return false;
    }

    // Copy elements to the local arguments //
    this->io = io;
    time = io->currentTime();

    // ROS Initialize
    nodeHandle = ros::NodeHandle(nodeNamespace);

    try {
        // ros plugin loader //
        if(!robotHWSimLoader){
            robotHWSimLoader = make_shared<pluginlib::ClassLoader<RobotHWSim<cnoid::ControllerIO*>>>(
                "choreonoid_ros",
                "hardware_interface::RobotHWSim<cnoid::ControllerIO*>");
        }

        // load RobotHWCnoid plugin
        if(!robotHWSim){
            robotHWSim = robotHWSimLoader->createInstance("hardware_interface/RobotHWCnoid");

            // load hardware_interface  //
            if(!robotHWSim->initSim(nodeHandle, io)){
                mv->putln(
                    format(_("The hardware interface of {0} cannot be initialized."), displayName()),
                    MessageView::Error);
            }
        }

        // register ros control manager //
        if(!controllerManager){
            controllerManager = make_shared<controller_manager::ControllerManager>(robotHWSim.get(), nodeHandle);
        }
    }
    catch(pluginlib::LibraryLoadException& ex){
        mv->putln(
            format(_("Failed to create robot simulation interface loader : {0}"), ex.what()),
            MessageView::Error);
        return false;
    }

    mv->putln(format(_("{0} has successfully been initialized"), displayName()));

    return true;
}


bool ROSControlItem::start()
{
    return true;
}


void ROSControlItem::input()
{
    
}


bool ROSControlItem::control()
{
    int last_sec = static_cast<int>(time);
    double last_nsec = (time - last_sec) * 1e9;
    int now_sec = static_cast<int>(io->currentTime());
    double now_nsec = (io->currentTime() - now_sec) * 1e9;
    
    ros::Time last(last_sec, static_cast<int>(last_nsec));
    ros::Time now(now_sec, static_cast<int>(now_nsec));
    ros::Duration period = now - last;
    
    robotHWSim->read(now, period);
    robotHWSim->write(now, period);
    
    controllerManager->update(now, period, false);
    
    time = io->currentTime();
    
    return true;
}


void ROSControlItem::output()
{

}

void ROSControlItem::stop()
{

}
