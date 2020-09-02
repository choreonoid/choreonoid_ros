/////////////////////////////////
/// @author Ryodo Tanaka
/////////////////////////////////

#include "cnoid_hw.h"
#include <transmission_interface/transmission_parser.h>
#include <cnoid/MessageView>
#include <sstream>

namespace hardware_interface {

bool CnoidHW::init(ros::NodeHandle& robot_nh, ros::NodeHandle& robot_hw_nh)
{
  nh_ = robot_hw_nh;
  bool ret = false;
  ret = loadURDF("robot_description");
  return ret;
}

bool CnoidHW::loadURDF(const std::string& param_name)
{
  using namespace std;
  using namespace cnoid;
  string urdf_string;
  stringstream ss;
  while(urdf_string.empty()) {
    string searched_param;
    if (nh_.searchParam(param_name, searched_param)) {      
      if(!nh_.getParam(searched_param, urdf_string)) {
        ss.str("");
        ss << "Waiting for URDF model." << endl;
        ss << "Finding parameter name : " << searched_param << endl;
        MessageView::instance()->put(ss.str(), MessageView::Warning);
        sleep(1);
      }
    }
    else {
      ss.str("");
      ss << "There are no parameter related to : " << param_name << endl;
      MessageView::instance()->put(ss.str(), MessageView::Error);
      return false;
    }
  }
  urdf_model_.initString(urdf_string);

  ss.str("");
  ss << "CnoidHW loaded the URDF model." << endl;
  MessageView::instance()->put(ss.str(), MessageView::Normal);

  // load transmissions //
  transmission_interface::TransmissionParser::parse(urdf_string, tmss_);

  ss.str("");
  ss << "CnoidHW loaded Transmission." << endl;
  MessageView::instance()->put(ss.str(), MessageView::Normal);
  
  return true;
}

void CnoidHW::read(const ros::Time& time, const ros::Duration& period)
{
}

void CnoidHW::write(const ros::Time& time, const ros::Duration& period)
{
}
} // namespace hardware_interface

PLUGINLIB_EXPORT_CLASS(hardware_interface::CnoidHW, hardware_interface::RobotHW)
