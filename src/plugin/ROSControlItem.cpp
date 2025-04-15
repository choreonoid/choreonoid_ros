/////////////////////////////////
/// @author Ryodo Tanaka
/////////////////////////////////

#include "ROSControlItem.h"
#include "Format.h"
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <cnoid/PutPropertyFunction>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using namespace hardware_interface;


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
            formatR(_("ROSControlItem \"{0}\" is invalid because it is not assigned to a body."),
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
    dt = timeStep();

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
                    formatR(_("The hardware interface of {0} cannot be initialized."), displayName()),
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
            formatR(_("Failed to create robot simulation interface loader : {0}"), ex.what()),
            MessageView::Error);
        return false;
    }

    mv->putln(formatR(_("{0} has successfully been initialized"), displayName()));

    return true;
}


bool ROSControlItem::start()
{
    return true;
}


void ROSControlItem::input()
{
    robotHWSim->read(ros::Time(io->currentTime()), ros::Duration(dt));
}


bool ROSControlItem::control()
{
    controllerManager->update(ros::Time(io->currentTime()), ros::Duration(dt), false);
    return true;
}


void ROSControlItem::output()
{
    robotHWSim->write(ros::Time(io->currentTime()), ros::Duration(dt));
}


void ROSControlItem::stop()
{

}
