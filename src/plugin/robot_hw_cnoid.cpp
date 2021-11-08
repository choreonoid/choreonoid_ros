/////////////////////////////////
/// @author Ryodo Tanaka
/////////////////////////////////

#include "robot_hw_cnoid.h"
// ROS //
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <transmission_interface/transmission_parser.h>
// Cnoid //
#include <cnoid/MessageView>
// STL //
#include <limits>
#include <sstream>
#include <unordered_map>

namespace hardware_interface
{
bool RobotHWCnoid::initSim(const ros::NodeHandle& nh, cnoid::ControllerIO* args)
{
  using namespace std;
  using namespace cnoid;
  namespace hi = hardware_interface;
  stringstream ss;

  nh_ = nh;
  io_ = args;

  if (!loadURDF("robot_description"))
    return false;

  dof_ = tmss_.size();
  joint_names_.resize(dof_);
  ctrl_types_.resize(dof_);
  data_.resize(dof_);
  command_.resize(dof_);

  joint_types_.resize(dof_);
  limits_.resize(dof_);

  unordered_map<string, unsigned int> if_map;
  if_map["PositionJointInterface"] = ControlType::POSITION;
  if_map["VelocityJointInterface"] = ControlType::VELOCITY;
  if_map["EffortJointInterface"] = ControlType::EFFORT;
  if_map["hardware_interface/PositionJointInterface"] = ControlType::POSITION;
  if_map["hardware_interface/VelocityJointInterface"] = ControlType::VELOCITY;
  if_map["hardware_interface/EffortJointInterface"] = ControlType::EFFORT;

  // Initialize values
  for (unsigned int i = 0; i < dof_; i++)
  {
    // Get joint_interfaces and check them //
    vector<string> joint_ifs = tmss_[i].joints_[0].hardware_interfaces_;
    if (tmss_[i].actuators_.empty() && tmss_[i].actuators_[0].hardware_interfaces_.empty())
    {
      ss.str("");
      ss << " should be nested inside the <joint> element, not <actuator>. ";
      ss << "The transmission will be properly loaded, but please update ";
      ss << "your robot model to remain compatible with future versions of the plugin." << endl;
      MessageView::instance()->put(ss.str(), MessageView::Warning);
    }
    if (joint_ifs.empty())
    {
      ss.str("");
      ss << tmss_[i].name_ << " on " << tmss_[i].joints_[0].name_;
      ss << " does not specify any hardware interface." << endl;
      MessageView::instance()->put(ss.str(), MessageView::Warning);
      continue;
    }
    else if (joint_ifs.size() > 1)
    {
      ss.str("");
      ss << tmss_[i].joints_[0].name_ << " of transmission ";
      ss << " specifies multiple hardware interfaces. ";
      ss << "Currently the default robot hardware simulation interface only supports one. Using the first entry"
         << endl;
      MessageView::instance()->put(ss.str(), MessageView::Warning);
    }

    // Get joint names //
    joint_names_[i] = tmss_[i].joints_[0].name_;

    // Get cnoid::Link //
    Link* link_tmp = io_->body()->link(joint_names_[i].c_str());
    Link* link = io_->body()->link(link_tmp->index());
    if (link == nullptr)
    {
      ss.str("");
      ss << "This robot has a joint named \"" << joint_names_[i];
      ss << "\" which is not in the choreonoid model." << endl;
      MessageView::instance()->put(ss.str(), MessageView::Error);
      return false;
    }
    links_.emplace_back(link);

    // Store current position, velocity, effort //
    data_[i].position = link->q();
    data_[i].velocity = link->dq();
    data_[i].effort = link->ddq();
    // Set commands by current data //
    command_[i].position = data_[i].position;
    command_[i].velocity = data_[i].velocity;
    command_[i].effort = data_[i].effort;

    // Create joint state interface //
    js_if_.registerHandle(
        hi::JointStateHandle(joint_names_[i], &data_[i].position, &data_[i].velocity, &data_[i].effort));

    // Set Control type from loaded cnoid::Link or hardware_interface //
    if (link->actuationMode())
    {
      link->setActuationMode(link->actuationMode());
      switch (link->actuationMode())
      {
        case Link::JOINT_ANGLE:
          ctrl_types_[i] = ControlType::POSITION;
          joint_handle_ = hi::JointHandle(js_if_.getHandle(joint_names_[i]), &command_[i].position);
          pj_if_.registerHandle(joint_handle_);
          break;
        case Link::JOINT_VELOCITY:
          ctrl_types_[i] = ControlType::VELOCITY;
          joint_handle_ = hi::JointHandle(js_if_.getHandle(joint_names_[i]), &command_[i].velocity);
          vj_if_.registerHandle(joint_handle_);
          break;
        case Link::JOINT_EFFORT:
          ctrl_types_[i] = ControlType::EFFORT;
          joint_handle_ = hi::JointHandle(js_if_.getHandle(joint_names_[i]), &command_[i].effort);
          ej_if_.registerHandle(joint_handle_);
          break;
        default:
          break;
      }
    }
    else
    {
      switch (if_map.at(joint_ifs[0]))
      {
        case ControlType::POSITION:
          ctrl_types_[i] = ControlType::POSITION;
          joint_handle_ = hi::JointHandle(js_if_.getHandle(joint_names_[i]), &command_[i].position);
          pj_if_.registerHandle(joint_handle_);
          link->setActuationMode(Link::JOINT_ANGLE);
          break;
        case ControlType::VELOCITY:
          ctrl_types_[i] = ControlType::VELOCITY;
          joint_handle_ = hi::JointHandle(js_if_.getHandle(joint_names_[i]), &command_[i].velocity);
          vj_if_.registerHandle(joint_handle_);
          link->setActuationMode(Link::JOINT_VELOCITY);
          break;
        case ControlType::EFFORT:
          ctrl_types_[i] = ControlType::EFFORT;
          joint_handle_ = hi::JointHandle(js_if_.getHandle(joint_names_[i]), &command_[i].effort);
          ej_if_.registerHandle(joint_handle_);
          link->setActuationMode(Link::JOINT_EFFORT);
          break;
        default:
          break;
      };
    }

    registerJointLimits(i);
  }

  // Register interfaces
  registerInterface(&js_if_);
  registerInterface(&pj_if_);
  registerInterface(&vj_if_);
  registerInterface(&ej_if_);

  return true;
}

bool RobotHWCnoid::loadURDF(const std::string& param_name)
{
  using namespace std;
  using namespace cnoid;
  string urdf_string;
  stringstream ss;
  while (urdf_string.empty())
  {
    string searched_param;
    if (nh_.searchParam(param_name, searched_param))
    {
      if (!nh_.getParam(searched_param, urdf_string))
      {
        ss.str("");
        ss << "Waiting for URDF model." << endl;
        ss << "Finding parameter name : " << searched_param << endl;
        MessageView::instance()->put(ss.str(), MessageView::Warning);
        sleep(1);
      }
    }
    else
    {
      ss.str("");
      ss << "There are no parameter related to : " << param_name << endl;
      MessageView::instance()->put(ss.str(), MessageView::Error);
      return false;
    }
  }
  urdf_model_ = make_shared<urdf::Model>();
  urdf_model_->initString(urdf_string);

  ss.str("");
  ss << "RobotHWCnoid loaded the URDF model." << endl;
  MessageView::instance()->put(ss.str(), MessageView::Normal);

  // load transmissions //
  transmission_interface::TransmissionParser::parse(urdf_string, tmss_);

  ss.str("");
  ss << "RobotHWCnoid loaded Transmission." << endl;
  MessageView::instance()->put(ss.str(), MessageView::Normal);

  return true;
}

bool RobotHWCnoid::registerJointLimits(const unsigned int& i)
{
  using namespace std;
  using namespace cnoid;
  stringstream ss;

  joint_types_[i] = urdf::Joint::UNKNOWN;
  limits_[i].lower.position = -numeric_limits<double>::max();
  limits_[i].upper.position = numeric_limits<double>::max();
  limits_[i].lower.velocity = 0;
  limits_[i].upper.velocity = numeric_limits<double>::max();
  limits_[i].lower.effort = 0;
  limits_[i].upper.effort = numeric_limits<double>::max();
  joint_limits_interface::JointLimits limits;
  bool has_limits = false;
  joint_limits_interface::SoftJointLimits soft_limits;
  bool has_soft_limits = false;

  if (urdf_model_ == nullptr)
  {
    ss.str("");
    ss << "URDF model is NULL. Abort." << endl;
    MessageView::instance()->put(ss.str(), MessageView::Error);
    return false;
  }
  else
  {
    const urdf::JointConstSharedPtr urdf_joint = urdf_model_->getJoint(joint_names_[i]);
    if (!urdf_joint)
    {
      ss.str("");
      ss << "URDF model joint is NULL. Abort." << endl;
      MessageView::instance()->put(ss.str(), MessageView::Error);
      return false;
    }
    else
    {
      joint_types_[i] = urdf_joint->type;
      // Get limits from the URDF file.
      if (joint_limits_interface::getJointLimits(urdf_joint, limits))
        has_limits = true;
      if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
        has_soft_limits = true;
    }
  }
  // Limits from parameter //
  if (joint_limits_interface::getJointLimits(joint_names_[i], nh_, limits))
    has_limits = true;

  // return if there are no limits //
  if (!has_limits)
    return false;

  // Check joint_type //
  if (joint_types_[i] == urdf::Joint::UNKNOWN)
  {
    if (limits.has_position_limits)
      joint_types_[i] = urdf::Joint::REVOLUTE;
    else
    {
      if (limits.angle_wraparound)
        joint_types_[i] = urdf::Joint::CONTINUOUS;
      else
        joint_types_[i] = urdf::Joint::PRISMATIC;
    }
  }

  // set limits //
  if (limits.has_position_limits)
  {
    limits_[i].lower.position = limits.min_position;
    limits_[i].upper.position = limits.max_position;
  }
  if (limits.has_velocity_limits)
    limits_[i].upper.velocity = limits.max_velocity;
  if (limits.has_effort_limits)
    limits_[i].upper.effort = limits.max_effort;

  // soft limits //
  if (has_soft_limits)
  {
    switch (ctrl_types_[i])
    {
      case ControlType::POSITION:
      {
        const joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(joint_handle_, limits, soft_limits);
        pj_lim_if_.registerHandle(limits_handle);
        break;
      }
      case ControlType::VELOCITY:
      {
        const joint_limits_interface::VelocityJointSoftLimitsHandle limits_handle(joint_handle_, limits, soft_limits);
        vj_lim_if_.registerHandle(limits_handle);
        break;
      }
      case ControlType::EFFORT:
      {
        const joint_limits_interface::EffortJointSoftLimitsHandle limits_handle(joint_handle_, limits, soft_limits);
        ej_lim_if_.registerHandle(limits_handle);
        break;
      }
      default:
        break;
    };
  }
  else
  {
    switch (ctrl_types_[i])
    {
      case ControlType::POSITION:
      {
        const joint_limits_interface::PositionJointSaturationHandle sat_handle(joint_handle_, limits);
        pj_sat_if_.registerHandle(sat_handle);
        break;
      }
      case ControlType::VELOCITY:
      {
        const joint_limits_interface::VelocityJointSaturationHandle sat_handle(joint_handle_, limits);
        vj_sat_if_.registerHandle(sat_handle);
        break;
      }
      case ControlType::EFFORT:
      {
        const joint_limits_interface::EffortJointSaturationHandle sat_handle(joint_handle_, limits);
        ej_sat_if_.registerHandle(sat_handle);
        break;
      }
      default:
        break;
    };
  }

  return true;
}

void RobotHWCnoid::read(const ros::Time& time, const ros::Duration& period)
{
  for (unsigned int i = 0; i < dof_; i++)
  {
    double position = links_[i]->q();
    if (joint_types_[i] == urdf::Joint::PRISMATIC)
      data_[i].position = position;
    else
      data_[i].position += angles::shortest_angular_distance(data_[i].position, position);

    data_[i].velocity = links_[i]->dq();
    data_[i].effort = links_[i]->ddq();
  }
}

void RobotHWCnoid::write(const ros::Time& time, const ros::Duration& period)
{
  pj_sat_if_.enforceLimits(period);
  pj_lim_if_.enforceLimits(period);
  vj_sat_if_.enforceLimits(period);
  vj_lim_if_.enforceLimits(period);
  ej_sat_if_.enforceLimits(period);
  ej_lim_if_.enforceLimits(period);

  for (unsigned int i = 0; i < dof_; i++)
  {
    switch (ctrl_types_[i])
    {
      case ControlType::POSITION:
      {
        links_[i]->q_target() = command_[i].position;
        break;
      }
      case ControlType::VELOCITY:
      {
        links_[i]->dq_target() = command_[i].velocity;
        break;
      }
      case ControlType::EFFORT:
      {
        links_[i]->u() = command_[i].effort;
        break;
      }
      default:
        break;
    };
  }
}

}  // namespace hardware_interface

PLUGINLIB_EXPORT_CLASS(hardware_interface::RobotHWCnoid, hardware_interface::RobotHWSim<cnoid::ControllerIO*>)
