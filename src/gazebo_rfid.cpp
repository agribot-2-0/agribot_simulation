/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <agribot_simulation/gazebo_rfid.h>
#include <gazebo/common/Events.hh>
#include <cstdlib>
#include <ctime>

constexpr uint8_t min_power_dbm = 17;
constexpr uint8_t max_power_dbm = 33;
constexpr uint8_t front_ant = 1;
constexpr uint8_t back_ant = 2;

namespace gazebo
{
GazeboRfid::GazeboRfid()
{

}

GazeboRfid::~GazeboRfid()
{
  node_handle_->shutdown();
  delete node_handle_;
}

auto GazeboRfid::calculateDrawingConditions(uint8_t power)
{
  struct conditions {
    float lower_condition, upper_condition = 0;
  };
  unsigned int seed = time(NULL);
  uint8_t rand_num = rand_r(&seed) % 101;
  return conditions{33, 66};
}

uint8_t GazeboRfid::drawNumberOfTags(uint8_t selected_antenna)
{
  uint8_t ant_pwr = 0;
  switch(selected_antenna)
  {
    case front_ant:
      ant_pwr = front_ant_pwr_;
      break;
    case back_ant:
      ant_pwr = back_ant_pwr_;
      break;
    default:
      break;
  }
  auto [cond1, cond2] = calculateDrawingConditions(ant_pwr);

}

bool GazeboRfid::scan(agribot_msgs::Tags::Request& req,   // NOLINT(runtime/references)
                      agribot_msgs::Tags::Response& res)  // NOLINT(runtime/references)
{
  int detected_tags = drawNumberOfTags(req.selected_antenna);

  //res.ids.push_back(i);
  return true;
}

bool GazeboRfid::setPower(agribot_msgs::RfidPower::Request& req,   // NOLINT(runtime/references)
                           agribot_msgs::RfidPower::Response& res)  // NOLINT(runtime/references)
{
  res.result = res.SUCCESS;
  if ((req.power_value > max_power_dbm) || (req.power_value < min_power_dbm))
  {
    ROS_INFO("Wrong PW value %d dB", req.power_value);
    res.result = res.FAIL;
    return true;
  }
  switch (req.selected_antenna)
  {
    case req.FRONT:
      front_ant_pwr_ = req.power_value;
      ROS_INFO("Front ant pw: %d dB", front_ant_pwr_);
      break;
    case req.BACK:
      back_ant_pwr_ = req.power_value;
      ROS_INFO("Back ant pw: %d dB", back_ant_pwr_);
      break;
    default:
      res.result = res.FAIL;
  }
  return true;
}

void GazeboRfid::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // load parameters
  if (_sdf->HasElement("robotNamespace"))
  {
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();
  }
  else
  {
    namespace_.clear();
  }
  if (!_sdf->HasElement("serviceName"))
  {
    service_ = "tags";
  }
  else
  {
    service_ = _sdf->GetElement("serviceName")->Get<std::string>();
  }
  // start ros node
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  }
  node_handle_ = new ros::NodeHandle("agribot/rfid/");
  rfid_scanner_service_ = node_handle_->advertiseService(service_, &GazeboRfid::scan, this);
  rfid_antenna_pw_service_ = node_handle_->advertiseService("antennas/power", &GazeboRfid::setPower, this);
}

void GazeboRfid::Reset()
{
}

void GazeboRfid::Update()
{
}
GZ_REGISTER_MODEL_PLUGIN(GazeboRfid)
}  // namespace gazebo
