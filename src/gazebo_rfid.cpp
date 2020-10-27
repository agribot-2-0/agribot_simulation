/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <agribot_simulation/gazebo_rfid.h>
#include <gazebo/common/Events.hh>
#include <cstdlib>
#include <ctime>

constexpr double min_power_dbm = 17.0;
constexpr double max_power_dbm = 33.0;
constexpr uint8_t tags_ids[] = { 0xaa, 0xbb };
constexpr int max_possible_number = 101;

// The boundaries below were selected in order to ensure
// specified chances of detecting tags, depending on
// antenna power:
// 1. ant_pw = 17db: * the probability for 1 or 2 tag is equal to 5%
//                     the probability for 0 tags is equal to 90%
// 2. ant_pw = 25db: * the probability for 0, 1 or 2 tags is equal to 33%
// 3. ant_pw = 33db: * the probability for 0 or 1 tag is equal to 5%
//                     the probability for 2 tags is equal to 90%
// The probabilities are calculated using quadrature function, taking
// into consideration three point specified above. The system of
// three equations was solved in order to achieve below parameters.
struct Parameters
{
  double a;
  double b;
  double c;
};
const Parameters low = { 0.226, -16.64, 307.41 };
const Parameters upper = { -0.21, 5.23, 66.97 };

namespace gazebo
{
GazeboRfid::GazeboRfid()
{
  // One of the antenna is set to max power in order to increase chance of
  // detecting two tags, and another is set to min power in order to increase
  // the chance of detecting zero tags. That way, the mechanizm of decreasing
  // and increasing the antenna power depending on number of detected tags
  // can be tested in agribot_tool_manager.
  front_ant_pwr_ = min_power_dbm;
  back_ant_pwr_ = max_power_dbm;
}

GazeboRfid::~GazeboRfid()
{
  node_handle_->shutdown();
  delete node_handle_;
}

double GazeboRfid::calculateSecondDegreePolynomial(double a, double b, double c, double x)
{
  return a * pow(x, 2) + b * x + c;
}

std::pair<double, double> GazeboRfid::calculateConditionsToDrawFrom(double ant_pwr)
{
  return std::make_pair(calculateSecondDegreePolynomial(low.a, low.b, low.c, ant_pwr),
                        calculateSecondDegreePolynomial(upper.a, upper.b, upper.c, ant_pwr));
}

std::size_t GazeboRfid::draw(auto lower_cond, auto upper_cond)
{
  std::size_t detected_tags_cnt = 0;
  unsigned int seed = time(NULL);
  uint8_t drawed_num = rand_r(&seed) % max_possible_number;
  ROS_INFO_STREAM("[GAZEBO RFID] Drawed number = " << static_cast<int>(drawed_num));

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

std::size_t GazeboRfid::drawNumberOfTags(double ant_pwr)
{
  auto [lower_cond, upper_cond] = calculateConditionsToDrawFrom(ant_pwr);
  ROS_INFO_STREAM("[GAZEBO RFID] lower_cond = " << lower_cond << " upper_cond = " << upper_cond);

  return draw(lower_cond, upper_cond);
}

double GazeboRfid::getPower(const agribot_msgs::Tags::Request& req)  // NOLINT(runtime/references)
{
  double pwr = 0;
  switch (req.selected_antenna)
  {
    case req.FRONT:
      pwr = front_ant_pwr_;
      break;
    case req.BACK:
      pwr = back_ant_pwr_;
      break;
    default:
      return false;
      break;
  }
  return pwr;
}

bool GazeboRfid::scan(agribot_msgs::Tags::Request& req,   // NOLINT(runtime/references)
                      agribot_msgs::Tags::Response& res)  // NOLINT(runtime/references)
{
  double pwr = getPower(req);
  std::size_t detected_tags = drawNumberOfTags(pwr);
  ROS_INFO_STREAM("[GAZEBO RFID] Tags detected = " << static_cast<int>(detected_tags));
  for (std::size_t i = 0; i < detected_tags; i++)
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
    ROS_INFO_STREAM("Wrong PW value " << static_cast<double>(req.power_value) << " dB");
    res.result = res.FAIL;
    return true;
  }
  switch (req.selected_antenna)
  {
    case req.FRONT:
      front_ant_pwr_ = static_cast<double>(req.power_value);
      ROS_INFO_STREAM("Setting front ant pw to " << front_ant_pwr_ << " dB");
      break;
    case req.BACK:
      back_ant_pwr_ = static_cast<double>(req.power_value);
      ROS_INFO_STREAM("Setting back ant pw to " << back_ant_pwr_ << " dB");
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
