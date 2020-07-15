#ifndef AGRIBOT_SENSORS_PLUGINS_GAZEBO_ROS_RFID_H
#define AGRIBOT_SENSORS_PLUGINS_GAZEBO_ROS_RFID_H

#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
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
  };
}
#endif // AGRIBOT_SENSORS_PLUGINS_GAZEBO_ROS_RFID_H