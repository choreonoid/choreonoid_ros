/////////////////////////////////
/// @author Ryodo Tanaka
/////////////////////////////////

#include "ROSControlItem.h"
#include "gettext.h"

#include <sstream>

#include <fmt/format.h>
#include <cnoid/Archive>
#include <cnoid/BodyItem>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/PutPropertyFunction>

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
  // stop();
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
  if (!archive.read("name space", namespace_))
    archive.read("name space", namespace_);

  return true;
}

void ROSControlItem::doPutProperties(PutPropertyFunction& putProperty)
{
  putProperty("Name space", namespace_, changeProperty(namespace_));
}

bool ROSControlItem::initialize(ControllerIO* io)
{
  using namespace std;
  using namespace cnoid;
  using namespace hardware_interface;

  stringstream ss;

  // Check body //
  if (!io->body())
  {
    ss.str("");
    ss << "ROSControlItem \"" << displayName() << "\" is invalid." << endl;
    ss << "Because it is not assigned to a body." << endl;
    MessageView::instance()->put(ss.str(), MessageView::Error);

    return false;
  }

  // Check that ROS has been initialized
  if (!ros::isInitialized())
  {
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

  try
  {
    // ros plugin loader //
    if (!rbt_hw_sim_loader_)
      rbt_hw_sim_loader_ = make_shared<pluginlib::ClassLoader<RobotHWSim<cnoid::ControllerIO*>>>("choreonoid_ros", "har"
                                                                                                                   "dwa"
                                                                                                                   "re_"
                                                                                                                   "int"
                                                                                                                   "erf"
                                                                                                                   "ace"
                                                                                                                   "::"
                                                                                                                   "Rob"
                                                                                                                   "otH"
                                                                                                                   "WSi"
                                                                                                                   "m<"
                                                                                                                   "cno"
                                                                                                                   "id:"
                                                                                                                   ":Co"
                                                                                                                   "ntr"
                                                                                                                   "oll"
                                                                                                                   "erI"
                                                                                                                   "O*"
                                                                                                                   ">");

    // load RobotHWCnoid plugin
    if (!rbt_hw_sim_)
    {
      rbt_hw_sim_ = rbt_hw_sim_loader_->createInstance("hardware_interface/RobotHWCnoid");

      // load hardware_interface  //
      if (!rbt_hw_sim_->initSim(nh_, io_))
      {
        ss.str("");
        ss << "Could not initialize robot simulation interface" << endl;
        MessageView::instance()->put(ss.str(), MessageView::Error);
      }
    }

    // register ros control manager //
    if (!manager_)
      manager_ = make_shared<controller_manager::ControllerManager>(rbt_hw_sim_.get(), nh_);
  }
  catch (pluginlib::LibraryLoadException& ex)
  {
    ss.str("");
    ss << "Failed to create robot simulation interface loader : " << ex.what() << endl;
    MessageView::instance()->put(ss.str(), MessageView::Error);
    return false;
  }

  ss.str("");
  ss << "Loaded cnoid::ROSControlItem" << endl;
  MessageView::instance()->put(ss.str(), MessageView::Normal);

  return true;
}

bool ROSControlItem::start(void)
{
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

  return true;
}

void ROSControlItem::output(void)
{
}

void ROSControlItem::stop(void)
{
}

}  // namespace cnoid
