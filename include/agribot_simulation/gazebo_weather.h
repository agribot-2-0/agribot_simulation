/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <agribot_msgs/AgribotWind.h>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/RelativeHumidity.h>
#include <sensor_msgs/Temperature.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <memory>
#include <string>
#include <vector>

namespace gazebo
{
class GazeboWeather : public ModelPlugin
{
public:
  GazeboWeather();
  virtual ~GazeboWeather();

  virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  void onUpdate();

private:
  // publishers
  ros::Publisher temperature_publisher_;
  ros::Publisher humidity_publisher_;
  ros::Publisher pressure_publisher_;
  ros::Publisher wind_publisher_;

  // messages
  sensor_msgs::Temperature msg_temp;
  sensor_msgs::RelativeHumidity msg_humi;
  sensor_msgs::FluidPressure msg_pressure;
  agribot_msgs::AgribotWind msg_wind;

  // gazebo
  physics::ModelPtr model;
  physics::WorldPtr world;
  gazebo::transport::NodePtr node_handle_;
  std::unique_ptr<ros::NodeHandle> rosnode_;
  event::ConnectionPtr update_connection_;
  common::Time last_update_time_;

  double update_period_;
};
}  // namespace gazebo
