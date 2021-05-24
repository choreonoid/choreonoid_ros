/////////////////////////////////
/// @author Ryodo Tanaka
/////////////////////////////////

#include "ROSControlItem.h"
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/ListControllers.h>
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
  using namespace hardware_interface;

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

  // Copy elements to the local arguments //
  io_ = io;
  body_ = io->body();
  tstep_ = io->worldTimeStep();
  time_ = io->currentTime();
  
  // ROS Initialize
  nh_ = ros::NodeHandle(namespace_);

  try {
    // ros plugin loader //
    if(rbt_hw_sim_loader_ == nullptr)
      rbt_hw_sim_loader_ = make_shared<pluginlib::ClassLoader<RobotHWSim<cnoid::ControllerIO*>>>("choreonoid_ros", "hardware_interface::RobotHWSim<cnoid::ControllerIO*>");
    if(rbt_hw_sim_ == nullptr) {
      rbt_hw_sim_ = rbt_hw_sim_loader_->createInstance("hardware_interface/RobotHWCnoid");
      // load hardware_interface  //
      if(!rbt_hw_sim_->initSim(nh_, io_)) {
        ss.str("");
        ss << "Could not initialize robot simulation interface" << endl;
        MessageView::instance()->put(ss.str(), MessageView::Error);
      }
    }
    // register ros control manager //
    if(manager_ == nullptr)
      manager_ = make_shared<controller_manager::ControllerManager>(rbt_hw_sim_.get(), nh_);
    else {
      for(auto c : controller_names_) {
        ros::ServiceClient sc = nh_.serviceClient<controller_manager_msgs::LoadController>("controller_manager/load_controller");
        controller_manager_msgs::LoadController load;
        load.request.name = c;
        sc.call(load);
      }
    }
  } catch(pluginlib::LibraryLoadException &ex) {
    ss.str("");
    ss << "Failed to create robot simulation interface loader : " << ex.what() << endl;
    MessageView::instance()->put(ss.str(), MessageView::Error);
  }
  
  ros::ServiceClient lcsc = nh_.serviceClient<controller_manager_msgs::ListControllers>("controller_manager/list_controllers");
  controller_manager_msgs::ListControllers lc;
  if(!lcsc.call(lc)) {
    ss.str("");
    ss << "Failed to lad cnoid::ROSControllerItem" << endl;
    MessageView::instance()->put(ss.str(), MessageView::Normal);
  } else { 
    for(auto c : lc.response.controller) {
      controller_names_.emplace_back(c.name);
      cout << c << endl;
    }
  }
  
  ss.str("");
  ss << "Loaded cnoid::ROSControlItem" << endl;
  MessageView::instance()->put(ss.str(), MessageView::Normal);

  std::cout << "You are in initialize()" << std::endl;
  
  return true;
}

bool ROSControlItem::start(void)
{
  std::cout << "You are in start()" << std::endl;
  
  // using namespace std;
  // using namespace cnoid;
  // using namespace hardware_interface;
  // stringstream ss;

  // nh_ = ros::NodeHandle(namespace_);

  // try {
  //   // ros plugin loader //
  //   rbt_hw_sim_loader_ = make_shared<pluginlib::ClassLoader<RobotHWSim<cnoid::ControllerIO*>>>("choreonoid_ros", "hardware_interface::RobotHWSim<cnoid::ControllerIO*>");
  //   rbt_hw_sim_ = rbt_hw_sim_loader_->createInstance("hardware_interface/RobotHWCnoid");
  //   // load hardware_interface  //
  //   if(!rbt_hw_sim_->initSim(nh_, io_)) {
  //     ss.str("");
  //     ss << "Could not initialize robot simulation interface" << endl;
  //     MessageView::instance()->put(ss.str(), MessageView::Error);
  //   }
  //   // register ros control manager //
  //   manager_ = make_shared<controller_manager::ControllerManager>(rbt_hw_sim_.get(), nh_);
  // } catch(pluginlib::LibraryLoadException &ex) {
  //   ss.str("");
  //   ss << "Failed to create robot simulation interface loader : " << ex.what() << endl;
  //   MessageView::instance()->put(ss.str(), MessageView::Error);
  // }

  // ss.str("");
  // ss << "Loaded cnoid::ROSControlItem" << endl;
  // MessageView::instance()->put(ss.str(), MessageView::Normal);
  
  return true;
}

void ROSControlItem::input(void)
{
}
bool ROSControlItem::control(void)
{
  int last_sec = static_cast<int>(time_);
  double last_nsec = (time_ - last_sec) * 1e9;
  int now_sec = static_cast<int>(io_->currentTime());
  double now_nsec = (io_->currentTime() - now_sec) * 1e9;
  
  ros::Time last(last_sec, static_cast<int>(last_nsec));
  ros::Time now(now_sec, static_cast<int>(now_nsec));
  ros::Duration period = now - last;

  rbt_hw_sim_->read(now, period);
  rbt_hw_sim_->write(now, period);

  manager_->update(now, period, false);

  
  time_ = io_->currentTime();
}

void ROSControlItem::output(void)
{
}

void ROSControlItem::stop(void)
{
  std::cout << "You are in stop()" << std::endl;

  for(auto c : controller_names_) {
    ros::ServiceClient sc = nh_.serviceClient<controller_manager_msgs::UnloadController>("controller_manager/unload_controller");
    controller_manager_msgs::UnloadController unload;
    unload.request.name = c;
    sc.call(unload);
  }

  
  rbt_hw_sim_.reset();
  rbt_hw_sim_loader_.reset();
//   manager_.reset();
}

} // namespace cnoid
