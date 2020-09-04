/////////////////////////////////
/// @author Ryodo Tanaka
/////////////////////////////////

#include "ROSControlItem.h"
#include <cnoid/MessageView>
#include <cnoid/PutPropertyFunction>
#include <cnoid/BodyItem>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <sstream>
#include <fmt/format.h>
#include "gettext.h"

namespace cnoid
{
void ROSControlItem::initializeClass(ExtensionManager* ext)
{
  using namespace std;
  using fmt::format;
  ext->itemManager().registerClass<ROSControlItem>(N_("ROSControlItem"));
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

ROSControlItem::~ROSControlItem()
{
  stop();
}

Item* ROSControlItem::doDuplicate(void) const
{
  return new ROSControlItem(*this);
}

bool ROSControlItem::store(Archive& archive)
{
  archive.write("name space", namespace_);

  return true;
}

bool ROSControlItem::restore(const Archive& archive)
{
  if(!archive.read("name space", namespace_))
    archive.read("name space", namespace_);
  
  return true;
}

void ROSControlItem::doPutProperties(PutPropertyFunction& putProperty)
{
  putProperty("Name space",
    namespace_, changeProperty(namespace_));
}

bool ROSControlItem::initialize(ControllerIO* io)
{
  using namespace std;
  using namespace cnoid;
  stringstream ss;

  // Check body //
  if (!io->body()) {
    ss.str("");
    ss << "ROSControlItem \"" << displayName() << "\" is invalid." << endl;
    ss << "Because it is not assigned to a body." << endl;
    MessageView::instance()->put(ss.str(), MessageView::Error);
    
    return false;
  }

  // Check that ROS has been initialized
  if(!ros::isInitialized()) {
    ss.str("");
    ss << "ROS master is not initialized." << endl;
    MessageView::instance()->put(ss.str(), MessageView::Warning);
    
    return false;
  }

  // Copy elements to the local arfuments //
  io_ = io;
  body_ = io->body();
  tstep_ = io->worldTimeStep();
  time_ = io->currentTime();

  return true;
}

bool ROSControlItem::start(void)
{
  using namespace std;
  using namespace hardware_interface;
  stringstream ss;

  nh_ = ros::NodeHandle(namespace_);

  try {
    // ros plugin loader //
    rbt_hw_sim_loader_ = make_shared<pluginlib::ClassLoader<RobotHWSim<cnoid::ControllerIO*>>>("choreonoid_ros", "hardware_interface::RobotHWSim<cnoid::ControllerIO*>");
    rbt_hw_sim_ = rbt_hw_sim_loader_->createInstance("hardware_interface/RobotHWCnoid");
    // load hardware_interface  //
    if(!rbt_hw_sim_->initSim(nh_, io_)) {
      ss.str("");
      ss << "Could not initialize robot simulation interface" << endl;
      MessageView::instance()->put(ss.str(), MessageView::Error);
    }
    // register ros control manager //
    manager_ = make_shared<controller_manager::ControllerManager>(rbt_hw_sim_.get(), nh_);
  } catch(pluginlib::LibraryLoadException &ex) {
    ss.str("");
    ss << "Failed to create robot simulation interface loader : " << ex.what() << endl;
    MessageView::instance()->put(ss.str(), MessageView::Error);
  }

  ss.str("");
  ss << "Loaded cnoid::ROSControlItem" << endl;
  MessageView::instance()->put(ss.str(), MessageView::Normal);
  
  return true;
}

void ROSControlItem::input(void)
{
  time_ = io_->currentTime();
  
  using namespace std;
  using namespace cnoid;
  stringstream ss;
  ss.str("");
  ss << "input() : " << time_ << endl;
  MessageView::instance()->put(ss.str(), MessageView::Normal);
}
bool ROSControlItem::control(void)
{
  time_ = io_->currentTime();
  
  using namespace std;
  using namespace cnoid;
  stringstream ss;
  ss.str("");
  ss << "control() : " << time_ << endl;
  MessageView::instance()->put(ss.str(), MessageView::Normal);
  
}

void ROSControlItem::output(void)
{
  time_ = io_->currentTime();
  
  using namespace std;
  using namespace cnoid;
  stringstream ss;
  ss.str("");
  ss << "output() : " << time_ << endl;
  MessageView::instance()->put(ss.str(), MessageView::Normal);

}

void ROSControlItem::stop(void)
{
}

} // namespace cnoid
