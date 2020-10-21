/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <agribot_simulation/gazebo_rfid.h>
#include <gazebo/common/Events.hh>
#include <cstdlib>
#include <ctime>

constexpr float min_power_dbm = 17.0;
constexpr float max_power_dbm = 33.0;
constexpr uint8_t front_ant = 1;
constexpr uint8_t back_ant = 2;
constexpr uint8_t tags_ids[] = { 0xaa, 0xbb };
constexpr int max_possible_number = 101;

// The boundaries below was selected in order to ensure
// specified chances of detecting tags, depending on
// antenna power:
// 1. ant_pw = 17db: * the probability for 1 or 2 tag is equal to 5%
//                     the probability for 0 tags is equal to 90%
// 2. ant_pw = 25db: * the probability for 0, 1 or 2 tags is equal to 33%
// 3. ant_pw = 33db: * the probability for 0 or 1 tag is equal to 5%
//                     the probability for 2 tags is equal to 90%
// The probabilities between specified points are calculated
// from quadrature function.
const struct
{
  float a;
  float b;
  float c;
} param_low = { 0.226, -16.64, 307.41 };

const struct
{
  float a;
  float b;
  float c;
} param_upper = { -0.21, 5.23, 66.97 };

namespace gazebo
{
GazeboRfid::GazeboRfid()
{
  front_ant_pwr_ = min_power_dbm;
  back_ant_pwr_ = max_power_dbm;
}

GazeboRfid::~GazeboRfid()
{
  node_handle_->shutdown();
  delete node_handle_;
}

float GazeboRfid::calculateSecondDegreePolynomial(float a, float b, float c, float x)
{
  return a * pow(x, 2) + b * x + c;
}

auto GazeboRfid::calculateConditionsToDrawFrom(float power)
{
  struct conditions
  {
    float l_cond = 0, u_cond = 0;
  };
  return conditions{ calculateSecondDegreePolynomial(param_low.a, param_low.b, param_low.c, power),
                     calculateSecondDegreePolynomial(param_upper.a, param_upper.b, param_upper.c, power) };
}

uint8_t GazeboRfid::drawNumberOfTags(uint8_t selected_antenna)
{
  uint8_t ant_pwr = 0;
  uint8_t detected_tags_cnt = 0;
  switch (selected_antenna)
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
  auto [lower_cond, upper_cond] = calculateConditionsToDrawFrom(ant_pwr);
  ROS_INFO("lower_cond = %f, upper_cond = %f", lower_cond, upper_cond);

  unsigned int seed = time(NULL);
  uint8_t drawed_num = rand_r(&seed) % max_possible_number;
  ROS_INFO("Drawed number = %d", drawed_num);
  if (drawed_num < lower_cond)
  {
    detected_tags_cnt = 0;
  }
  if (drawed_num >= lower_cond && drawed_num <= upper_cond)
  {
    detected_tags_cnt = 1;
  }
  if (drawed_num > upper_cond)
  {
    detected_tags_cnt = 2;
  }
  return detected_tags_cnt;
}

bool GazeboRfid::scan(agribot_msgs::Tags::Request& req,   // NOLINT(runtime/references)
                      agribot_msgs::Tags::Response& res)  // NOLINT(runtime/references)
{
  int detected_tags = drawNumberOfTags(req.selected_antenna);
  ROS_INFO("Tags detected = %d", detected_tags);
  for (int i = 0; i < detected_tags; i++)
  {
    res.ids.push_back(tags_ids[i]);
  }
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
      front_ant_pwr_ = static_cast<float>(req.power_value);
      ROS_INFO("Setting front ant pw to %f dB", front_ant_pwr_);
      break;
    case req.BACK:
      back_ant_pwr_ = static_cast<float>(req.power_value);
      ROS_INFO("Setting back ant pw to %f dB ", back_ant_pwr_);
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
