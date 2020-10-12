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

void GazeboWeather::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->model = _parent;
  this->world = _parent->GetWorld();

  node_handle_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node_handle_->Init();

  rosnode_ = std::make_unique<ros::NodeHandle>(ros::NodeHandle("weather_station"));
  temperature_publisher_ = rosnode_->advertise<sensor_msgs::Temperature>("temperature", 1);
  humidity_publisher_ = rosnode_->advertise<sensor_msgs::RelativeHumidity>("humidity", 1);
  pressure_publisher_ = rosnode_->advertise<sensor_msgs::FluidPressure>("atmospheric_pressure", 1);
  wind_publisher_ = rosnode_->advertise<agribot_msgs::AgribotWind>("wind_speed", 1);

  last_update_time_ = this->world->SimTime();

  msg_temp.variance = 1;
  msg_humi.variance = 2;
  msg_pressure.variance = 1;
  msg_wind.speed_variance = 0.1;

  update_period_ = 1;  // seconds

  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboWeather::onUpdate, this));
}

void GazeboWeather::onUpdate()
{
  common::Time current_time = this->world->SimTime();
  double seconds_since_last_update = (current_time - last_update_time_).Double();
  if (seconds_since_last_update > update_period_)
  {
    msg_temp.header.stamp = ros::Time::now();
    msg_humi.header.stamp = ros::Time::now();
    msg_pressure.header.stamp = ros::Time::now();
    msg_wind.header.stamp = ros::Time::now();

    msg_temp.temperature = this->world->Atmosphere().Temperature();
    msg_pressure.fluid_pressure = this->world->Atmosphere().Pressure();
    msg_wind.average_speed = this->world->Wind().LinearVel().SquaredLength();

    temperature_publisher_.publish(msg_temp);
    humidity_publisher_.publish(msg_humi);
    pressure_publisher_.publish(msg_pressure);
    wind_publisher_.publish(msg_wind);

    last_update_time_ = current_time;
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboWeather)
}  // namespace gazebo
