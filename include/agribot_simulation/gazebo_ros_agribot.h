#ifndef AGRIBOT_PLUGIN_HH
#define AGRIBOT_PLUGIN_HH

#include <map>

// Gazebo
// #include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>

// ROS
#include <ros/ros.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>
#include <agribot_msgs/AgribotCaterpillar.h>
// #include <geometry_msgs/Pose2D.h>
// #include <nav_msgs/Odometry.h>
// #include <sensor_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

  class Joint;
  class Entity;

  class GazeboRosAgribot : public ModelPlugin {

    enum OdomSource
    {
        ENCODER = 0,
        WORLD = 1,
    };
    public:
      GazeboRosAgribot();
      ~GazeboRosAgribot();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void onUpdate();

    private:
    //   void publishOdometry(double step_time);
    //   void getWheelVelocities();
    //   void publishWheelTF(); /// publishes the wheel tf's
    //   void publishWheelJointState();
    //   void UpdateOdometryEncoder();
      void CreatePubsAndSubs();
      void UpdateVelocities();


      GazeboRosPtr gazebo_ros_;
      physics::ModelPtr model_;
      event::ConnectionPtr update_connection_;

      // double wheel_separation_;
      double wheel_diameter_;
      double wheel_torque;
      double wheel_speed_[4];
      double wheel_accel;
      double wheel_speed_instr_[4];

      std::vector<physics::JointPtr> joints_;
      std::vector<std::string> joints_name_;

      // ROS STUFF
      gazebo::transport::NodePtr node_handle_;
      gazebo::transport::SubscriberPtr cmd_vel_subscriber_;

      boost::mutex lock;

      // std::string robot_namespace_;
      std::string command_sub_topic_;
      // std::string odometry_topic_;
      // std::string odometry_frame_;
      // std::string robot_base_frame_;
      // bool publish_tf_;

      // Custom Callback Queue
      // ros::CallbackQueue queue_;
      // boost::thread callback_queue_thread_;
      // void QueueThread();

      // DiffDrive stuff
      void cmdVelCallback(const agribot_msgs::AgribotCaterpillar::ConstPtr& cmd_msg);

      // double x_;
      // double rot_;
      // bool alive_;
      //
      // // Update Rate
      double update_rate_;
      // double update_period_;
      // common::Time last_update_time_;
      //
      // OdomSource odom_source_;
      // geometry_msgs::Pose2D pose_encoder_;
      // common::Time last_odom_update_;

    // Flags
    bool publishWheelTF_;
    bool publishOdomTF_;
    bool publishWheelJointState_;

  };

}

#endif
