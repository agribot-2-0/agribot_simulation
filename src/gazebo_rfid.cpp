/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <agribot_simulation/gazebo_rfid.h>
#include <gazebo/common/Events.hh>

#define TAG_FRONT 0xaa
#define TAG_BACK 0xbb

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

bool GazeboRfid::bScan(agribot_msgs::Tags::Request& req,   // NOLINT(runtime/references)
                       agribot_msgs::Tags::Response& res)  // NOLINT(runtime/references)
{
  if (req.selected_antenna == req.FRONT)
  {
    res.ids.push_back(TAG_FRONT);
  }
  if (req.selected_antenna == req.BACK)
  {
    res.ids.push_back(TAG_BACK);
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
  node_handle_ = new ros::NodeHandle("hal_rfid");
  rfid_service_ = node_handle_->advertiseService(service_, &GazeboRfid::bScan, this);
  ROS_ERROR("HELLO FROM RFID");
}

void GazeboRfid::Reset()
{
}

void GazeboRfid::Update()
{
}
GZ_REGISTER_MODEL_PLUGIN(GazeboRfid)
}  // namespace gazebo
