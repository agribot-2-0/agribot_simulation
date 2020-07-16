/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <gazebo/common/Plugin.hh>

#include <agribot_msgs/Tags.h>
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
  physics::WorldPtr world;
  ros::NodeHandle* node_handle_;
  std::string namespace_;
  std::string service_;
  ros::ServiceServer rfid_service_;
  std::vector<uint16_t> tags_;

  int gen_random_int(int min, int max);
  bool scan(agribot_msgs::Tags::Request& req, agribot_msgs::Tags::Response& res);  // NOLINT(runtime/references)
};
}  // namespace gazebo
