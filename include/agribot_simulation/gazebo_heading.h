#ifndef AGRIBOT_PLUGIN_HH
#define AGRIBOT_PLUGIN_HH

#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

  class Link;
  class Entity;

  class GazeboHeading : public ModelPlugin {
    public:
      GazeboHeading();
      ~GazeboHeading();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void onUpdate();

    private:
      void publishHeading();
      void CreatePubsAndSubs();

      GazeboRosPtr gazebo_ros_;
      physics::WorldPtr world;
      physics::ModelPtr model_;
      event::ConnectionPtr update_connection_;

      physics::LinkPtr link_;
      std::string link_name;

      // ROS STUFF
      ros::NodeHandle* rosnode_;
      ros::Publisher heading_publisher_;
      gazebo::transport::NodePtr node_handle_;
      sensor_msgs::Imu heading_;

      boost::mutex lock;

      std::string robot_namespace_;
      std::string heading_topic_;
      
      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      bool alive_;
      //
      // // Update Rate
      double update_rate_;
      double update_period_;
      common::Time last_update_time_;
  };

}

#endif
