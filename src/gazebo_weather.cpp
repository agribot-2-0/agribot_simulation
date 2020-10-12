/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <agribot_simulation/gazebo_weather.h>

#include <gazebo/common/Events.hh>
#include <gazebo/physics/Model.hh>

// c++
#include <cstdlib>
#include <ctime>
#include <memory>

namespace gazebo
{
GazeboWeather::GazeboWeather()
{
}

GazeboWeather::~GazeboWeather()
{
}

void GazeboWeather::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  this->model_ = parent;
  this->world_ = parent->GetWorld();

  node_handle_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node_handle_->Init();

  rosnode_ = std::make_unique<ros::NodeHandle>(ros::NodeHandle("weather_station"));
  temperature_publisher_ = rosnode_->advertise<sensor_msgs::Temperature>("temperature", 1);
  humidity_publisher_ = rosnode_->advertise<sensor_msgs::RelativeHumidity>("humidity", 1);
  pressure_publisher_ = rosnode_->advertise<sensor_msgs::FluidPressure>("atmospheric_pressure", 1);
  wind_publisher_ = rosnode_->advertise<agribot_msgs::AgribotWind>("wind_speed", 1);

  last_update_time_ = this->world_->SimTime();

  msg_temp_.variance = 1;
  msg_humi_.variance = 2;
  msg_pressure_.variance = 1;
  msg_wind_.speed_variance = 0.1;

  update_period_ = 1;  // seconds

  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboWeather::onUpdate, this));
}

void GazeboWeather::onUpdate()
{
  common::Time current_time = this->world_->SimTime();
  double seconds_since_last_update = (current_time - last_update_time_).Double();
  if (seconds_since_last_update > update_period_)
  {
    msg_temp_.header.stamp = ros::Time::now();
    msg_humi_.header.stamp = ros::Time::now();
    msg_pressure_.header.stamp = ros::Time::now();
    msg_wind_.header.stamp = ros::Time::now();

    msg_temp_.temperature = kelvinToCelsius(this->world_->Atmosphere().Temperature());
    msg_pressure_.fluid_pressure = this->world_->Atmosphere().Pressure();
    msg_wind_.average_speed = this->world_->Wind().LinearVel().SquaredLength();

    temperature_publisher_.publish(msg_temp_);
    humidity_publisher_.publish(msg_humi_);
    pressure_publisher_.publish(msg_pressure_);
    wind_publisher_.publish(msg_wind_);

    last_update_time_ = current_time;
  }
}

double GazeboWeather::kelvinToCelsius(double temp_in_kelvin)
{
  return temp_in_kelvin - 272.15;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboWeather)
}  // namespace gazebo
