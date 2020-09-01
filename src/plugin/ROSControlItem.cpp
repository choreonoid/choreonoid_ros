/////////////////////////////////
/// @author Ryodo Tanaka
/////////////////////////////////

#include "ROSControlItem.h"
#include <cnoid/MessageView>
#include <cnoid/PutPropertyFunction>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <sstream>

namespace cnoid
{
void ROSControlItem::initializeClass(ExtensionManager* ext)
{
  ext->itemManager().registerClass<ROSControlItem>("ROSControlItem");
  ext->itemManager().addCreationPanel<ROSControlItem>();
}

ROSControlItem::ROSControlItem(void)
{
  io_ = nullptr;
}

ROSControlItem::ROSControlItem(const ROSControlItem& org) : ControllerItem(org)
{
    io_ = nullptr;
}

Item* ROSControlItem::doDuplicate(void) const
{
  return new ROSControlItem(*this);
}

bool ROSControlItem::store(Archive& archive)
{
  archive.write("name space", namespace_);
  archive.write("robot_descripton", robot_description_);

  return true;
}

bool ROSControlItem::restore(const Archive& archive)
{
  if(!archive.read("name space", namespace_))
    archive.read("name space", namespace_);
  
  if(!archive.read("robot_description", robot_description_))
    archive.read("robot_escription", robot_description_);

  return true;
}

void ROSControlItem::doPutProperties(PutPropertyFunction& putProperty)
{
  putProperty("Name space for the program.",
    namespace_, changeProperty(namespace_));
  
  putProperty("Robot description parameter name.",
    robot_description_, changeProperty(robot_description_));
}

bool ROSControlItem::initialize(ControllerIO* io)
{
  using namespace std;
  using namespace cnoid;
  stringstream ss;

  // Check body //
  if (!io->body()) {
    ss.clear();
    ss << "ROSControlItem \"" << displayName() << "\" is invalid." << endl;
    ss << "Because it is not assigned to a body." << endl;
    MessageView::instance()->put(ss.str(), MessageView::Error);
    
    return false;
  }

  // Check that ROS has been initialized
  if(!ros::isInitialized()) {
    ss.clear();
    ss << "ROS master is not initialized." << endl;
    MessageView::instance()->put(ss.str(), MessageView::Warning);
    
    return false;
  }
  
  io_ = io;
  body_ = io->body();
  tstep_ = io->worldTimeStep();
  time_ = io->currentTime();

  return true;
}

bool ROSControlItem::start(void)
{
  using namespace std;
  using namespace cnoid;
  stringstream ss;

  nh_ = ros::NodeHandle(namespace_);

  try {
    // ros plugin loader //
    rbt_hw_sim_loader_ = make_shared<pluginlib::ClassLoader<RobotHWSim>>("choreonoid_ros", "cnoid::RobotHWSim");

    //  rbt_hw_sim_ = to_std<RobotHWSim>(rbt_hw_sim_loader_->createInstance("cnoid/RobotHWSim"));
    // load cnoid::RobotHWSim //
    if(!rbt_hw_sim_->initSim(nh_, robot_description_)) {
      ss.clear();
      ss << "Could not initialize robot simulation interface" << endl;
      MessageView::instance()->put(ss.str(), MessageView::Error);
    }
    // register ros control manager //
    manager_ = make_shared<controller_manager::ControllerManager>(rbt_hw_sim_.get(), nh_);
  }
  catch(pluginlib::LibraryLoadException &ex) {
    ss.clear();
    ss << "Failed to create robot simulation interface loader : " << ex.what() << endl;
    MessageView::instance()->put(ss.str(), MessageView::Error);
  }

  ss.clear();
  ss << "Loaded cnoid::ROSControlItem" << endl;
  MessageView::instance()->put(ss.str(), MessageView::Normal);
  
  return true;
}

void ROSControlItem::input(void)
{
}
bool ROSControlItem::control(void)
{
}

void ROSControlItem::output(void)
{
}

void ROSControlItem::stop(void)
{
}

} // namespace cnoid
