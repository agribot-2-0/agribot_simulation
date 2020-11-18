/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

// Gazebo
#include <gazebo_plugins/gazebo_ros_utils.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

// ROS
#include <agribot_msgs/AgribotCaterpillar.h>
#include <agribot_msgs/AgribotCaterpillarWheelsRotation.h>
#include <agribot_msgs/AgribotTool.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// Custom Callback Queue
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>

#include <map>
#include <string>
#include <vector>

// Boost
#include <boost/bind.hpp>
#include <boost/thread.hpp>

namespace gazebo
{
class Joint;
class Entity;

class GazeboRosAgribot : public ModelPlugin
{
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
  void publishOdometry(double step_time);
  void publishEncoder();
  void publishCarts();
  void publishTools();
  void CreatePubsAndSubs();
  void UpdateVelocities();

  GazeboRosPtr gazebo_ros_;
  physics::WorldPtr world;
  physics::ModelPtr model_;
  event::ConnectionPtr update_connection_;

  double wheel_separation_;
  double wheel_radius_;
  double chasis_length_;
  double wheel_torque;
  double wheel_speed_[4];
  double cart_speed_[4];
  double cart_angle_[2];
  double wheel_accel;
  double wheel_speed_instr_[4];

  std::vector<physics::JointPtr> joints_;
  std::vector<std::string> joints_name_;

  // ROS STUFF
  ros::NodeHandle* rosnode_;
  ros::Subscriber cmd_vel_subscriber_;
  ros::Subscriber cmd_front_tool_subscriber_;
  ros::Subscriber cmd_back_tool_subscriber_;
  ros::Publisher odometry_publisher_;
  ros::Publisher encoder_publisher_;
  ros::Publisher front_cart_publisher_;
  ros::Publisher back_cart_publisher_;
  ros::Publisher catterpilar_wheels_publisher_;
  ros::Publisher front_tool_publisher_;
  ros::Publisher back_tool_publisher_;
  gazebo::transport::NodePtr node_handle_;
  tf::TransformBroadcaster* transform_broadcaster_;
  agribot_msgs::AgribotCaterpillar encoder_;
  std_msgs::Float32 FrontAngleMsg_;
  std_msgs::Float32 BackAngleMsg_;
  agribot_msgs::AgribotCaterpillarWheelsRotation CaterpillarEncoderMsg_;
  nav_msgs::Odometry odom_;
  std::string tf_prefix_;
  bool broadcast_tf_;

  boost::mutex lock;

  std::string robot_namespace_;
  std::string command_sub_topic_;
  std::string odometry_topic_;
  std::string encoder_topic_;
  std::string odometry_frame_;
  std::string robot_base_frame_;

  // Custom Callback Queue
  ros::CallbackQueue queue_;
  boost::thread callback_queue_thread_;
  void QueueThread();

  // DiffDrive stuff
  void cmdVelCallback(const agribot_msgs::AgribotCaterpillar::ConstPtr& cmd_msg);
  // Tools callbacks
  void cmdToolsFrontCallback(const agribot_msgs::AgribotTool::ConstPtr& tool_msg);
  void cmdToolsBackCallback(const agribot_msgs::AgribotTool::ConstPtr& tool_msg);

  bool alive_;
  //
  // // Update Rate
  double update_rate_;
  double update_period_;
  common::Time last_update_time_;
  //
  OdomSource odom_source_;
  geometry_msgs::Pose2D pose_encoder_;
  common::Time last_odom_update_;
  // Tools msgs:
  agribot_msgs::AgribotTool front_cart_tool_encoder_;
  agribot_msgs::AgribotTool back_cart_tool_encoder_;

  double covariance_x_;
  double covariance_y_;
  double covariance_yaw_;

  // Flags
  bool publishJointTF_;
  bool publishOdomTF_;
  bool publishWheelJointState_;
};

}  // namespace gazebo
