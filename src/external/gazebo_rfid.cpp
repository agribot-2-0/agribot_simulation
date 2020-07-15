/*
* Copyright (C) 2020 Agribot. All rights reserved.
*/

#include <agribot_simulation/external/gazebo_rfid.h>
#include <gazebo/common/Events.hh>


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

  bool GazeboRfid::bScanForTags(std::vector<std::uint16_t>& vDiscoveredTags)
  {
    vDiscoveredTags.push_back(2211);
    return true;
  }

  bool GazeboRfid::bScan(agribot_msgs::Tags::Request &req,  // NOLINT(runtime/references)
                         agribot_msgs::Tags::Response &res)  // NOLINT(runtime/references)
  {
    return bScanForTags(res.ids);
  }

  void GazeboRfid::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // load parameters
    if (_sdf->HasElement("robotNamespace"))
      namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();
    else
      namespace_.clear();

    if (!_sdf->HasElement("serviceName"))
      service_ = "hal_rfid/tags";
    else
      service_ = _sdf->GetElement("serviceName")->Get<std::string>();

    // start ros node
    if (!ros::isInitialized())
    {
      int argc = 0;
      char** argv = NULL;
      ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
    }
    node_handle_ = new ros::NodeHandle("hal_rfid");
    node_handle_->advertiseService(service_, GazeboRfid::bScan);
    ROS_ERROR("HELLO FROM RFID");
  }

  void GazeboRfid::Reset()
  {
  }

  void GazeboRfid::Update()
  {
  }
  GZ_REGISTER_MODEL_PLUGIN(GazeboRfid)
}