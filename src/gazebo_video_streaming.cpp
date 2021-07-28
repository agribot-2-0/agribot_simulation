/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <agribot_simulation/gazebo_video_streaming.h>

#include <gazebo/common/Events.hh>
#include <gazebo/physics/Model.hh>

// c++
#include <cstdlib>
#include <ctime>
#include <memory>

namespace gazebo
{
GazeboVideoStreaming::GazeboVideoStreaming()
{
}

GazeboVideoStreaming::~GazeboVideoStreaming()
{
}

void GazeboVideoStreaming::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  this->model_ = parent;
  this->world_ = parent->GetWorld();

  rosnode_ = std::make_unique<ros::NodeHandle>(ros::NodeHandle("hal_video_stream"));

  start_stream_service_ =
      rosnode_->advertiseService("start_stream", &GazeboVideoStreaming::callback_start_stream, this);
  stop_stream_service_ = rosnode_->advertiseService("stop_stream", &GazeboVideoStreaming::callback_stop_stream, this);
}

bool GazeboVideoStreaming::callback_start_stream(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO_STREAM("[GazeboVideoStreaming] Video streaming started.");
  return true;
}

bool GazeboVideoStreaming::callback_stop_stream(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO_STREAM("[GazeboVideoStreaming] Video streaming stopped.");
  return true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboVideoStreaming)
}  // namespace gazebo
