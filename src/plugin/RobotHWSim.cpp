/////////////////////////////////
/// @author Ryodo Tanaka
/////////////////////////////////

#include "RobotHWSim.h"
#include <transmission_interface/transmission_parser.h>
#include <cnoid/MessageView>
#include <sstream>

namespace cnoid{

bool RobotHWSim::initSim(ros::NodeHandle& nh, const std::string& param_name)
{
  nh_ = nh;
  bool ret = false;
  ret = loadURDF(param_name);
}

bool RobotHWSim::loadURDF(const std::string& param_name)
{
  using namespace std;
  using namespace cnoid;
  string urdf_string;
  stringstream ss;
  while(urdf_string.empty()) {
    string search_param_name;
    if (nh_.searchParam(param_name, search_param_name)) {
      ss.clear();
      ss << "Waiting for URDF model." << endl;
      ss << "Finding parameter name : " << search_param_name << endl;
      MessageView::instance()->put(ss.str(), MessageView::Warning);
      
      nh_.getParam(search_param_name, urdf_string);
      sleep(1);
    }
    else {
      ss.clear();
      ss << "There are no parameter name : " << search_param_name << endl;
      MessageView::instance()->put(ss.str(), MessageView::Error);
      return false;
    }
  }
  urdf_model_.initString(urdf_string);

  ss.clear();
  ss << "cnoid::RobotHWSim loaded the URDF model." << endl;
  MessageView::instance()->put(ss.str(), MessageView::Normal);

  // load transmissions //
  transmission_interface::TransmissionParser::parse(urdf_string, tmss_);

  ss.clear();
  ss << "cnoid::RobotHWSim loaded Transmission." << endl;
  MessageView::instance()->put(ss.str(), MessageView::Normal);
  
  return true;
}

void RobotHWSim::read(const ros::Time& time, const ros::Duration& period)
{
}

void RobotHWSim::write(const ros::Time& time, const ros::Duration& period)
{
}
} // namespace cnoid

PLUGINLIB_EXPORT_CLASS(cnoid::RobotHWSim, hardware_interface::RobotHW)
