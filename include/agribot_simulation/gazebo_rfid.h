/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <gazebo/common/Plugin.hh>

#include <agribot_msgs/Tags.h>
#include <agribot_msgs/RfidPower.h>
#include <hector_gazebo_plugins/sensor_model.h>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace gazebo
{
class GazeboRfid : public ModelPlugin
{
public:
  GazeboRfid();
  virtual ~GazeboRfid();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Reset();
  virtual void Update();

private:
  static constexpr uint8_t vec_size_ = 10;
  physics::WorldPtr world;
  ros::NodeHandle* node_handle_;
  std::string namespace_;
  std::string service_;
  ros::ServiceServer rfid_scanner_service_;
  ros::ServiceServer rfid_antenna_pw_service_;
  std::vector<uint16_t> tags_;
  uint8_t front_ant_pwr_;
  uint8_t back_ant_pwr_;

  auto calculateDrawingConditions(uint8_t power);
  uint8_t drawNumberOfTags(uint8_t selected_antenna);
  bool scan(agribot_msgs::Tags::Request& req, agribot_msgs::Tags::Response& res);                 // NOLINT(runtime/references)
  bool setPower(agribot_msgs::RfidPower::Request& req, agribot_msgs::RfidPower::Response& res);  // NOLINT(runtime/references)
};
}  // namespace gazebo
