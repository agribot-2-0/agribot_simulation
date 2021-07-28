/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <std_srvs/Empty.h>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <ros/ros.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <memory>
#include <string>
#include <vector>

namespace gazebo
{
class GazeboVideoStreaming : public ModelPlugin
{
public:
  GazeboVideoStreaming();
  virtual ~GazeboVideoStreaming();

  virtual void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

private:
  ros::ServiceServer start_stream_service_;
  ros::ServiceServer stop_stream_service_;

  // messages
  std_srvs::Empty msg_;

  // gazebo
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  std::unique_ptr<ros::NodeHandle> rosnode_;
  event::ConnectionPtr update_connection_;

  bool callback_start_stream(std_srvs::Empty::Request& req,    // NOLINT(runtime/references)
                             std_srvs::Empty::Response& res);  // NOLINT(runtime/references)
  bool callback_stop_stream(std_srvs::Empty::Request& req,     // NOLINT(runtime/references)
                            std_srvs::Empty::Response& res);   // NOLINT(runtime/references)
};
}  // namespace gazebo
