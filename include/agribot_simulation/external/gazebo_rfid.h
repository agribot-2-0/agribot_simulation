/*
* Copyright (C) 2020 Agribot. All rights reserved.
*/

#ifndef AGRIBOT_SENSORS_PLUGINS_GAZEBO_ROS_RFID_H
#define AGRIBOT_SENSORS_PLUGINS_GAZEBO_ROS_RFID_H

#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include <agribot_msgs/Tags.h>
#include <hector_gazebo_plugins/sensor_model.h>

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
    ros::NodeHandle* node_handle_;
    std::string namespace_;
    std::string service_;

    static bool bScanForTags(std::vector<std::uint16_t>& vDiscoveredTags);
    static bool bScan(agribot_msgs::Tags::Request &req, agribot_msgs::Tags::Response &res);
  };
}
#endif // AGRIBOT_SENSORS_PLUGINS_GAZEBO_ROS_RFID_H