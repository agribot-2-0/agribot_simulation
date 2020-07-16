/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <agribot_simulation/gazebo_rfid.h>
#include <gazebo/common/Events.hh>
#include <cstdlib>
#include <ctime>

namespace gazebo
{
GazeboRfid::GazeboRfid()
{
  for (uint16_t i = 0; i < vec_size_; ++i)
  {
    tags_.push_back(i);
  }
}

GazeboRfid::~GazeboRfid()
{
  node_handle_->shutdown();
  delete node_handle_;
}

int GazeboRfid::gen_random_int(int min, int max)
{
  unsigned int seed = time(NULL);
  return (rand_r(&seed) % (max + 1)) + min;
}

bool GazeboRfid::scan(agribot_msgs::Tags::Request& req,   // NOLINT(runtime/references)
                      agribot_msgs::Tags::Response& res)  // NOLINT(runtime/references)
{
  int rand_ind = gen_random_int(0, tags_.size());
  if (rand_ind == tags_.size())
  {
    return true;
  }
  res.ids.push_back(tags_[rand_ind]);
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
  rfid_service_ = node_handle_->advertiseService(service_, &GazeboRfid::scan, this);
}

void GazeboRfid::Reset()
{
}

void GazeboRfid::Update()
{
}
GZ_REGISTER_MODEL_PLUGIN(GazeboRfid)
}  // namespace gazebo
