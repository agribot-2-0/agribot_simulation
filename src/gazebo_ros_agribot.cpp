/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <agribot_simulation/gazebo_ros_agribot.h>
#include <assert.h>
#include <math.h>
#include <ros/ros.h>

#include <algorithm>

#define RAD_TO_DEG (180 / M_PI)
#define DEG_TO_RAD (M_PI / 180)
constexpr int joints_cnt = 12;

// #define DEBUG_VEL
// #define DEBUG_TF

namespace gazebo
{
enum
{
  FRONT_RIGHT_WHEEL,
  FRONT_LEFT_WHEEL,
  BACK_RIGHT_WHEEL,
  BACK_LEFT_WHEEL,
  FRONT_CART,
  BACK_CART,
  FRONT_LOWER_HIT_ANG,
  FRONT_UPPER_HIT_ANG,
  FRONT_UPPER_HIT_HEI,
  BACK_LOWER_HIT_ANG,
  BACK_UPPER_HIT_ANG,
  BACK_UPPER_HIT_HEI
};

GazeboRosAgribot::GazeboRosAgribot()
{
}

// Destructor
GazeboRosAgribot::~GazeboRosAgribot()
{
}

// Load the controller
void GazeboRosAgribot::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;
  this->world = _model->GetWorld();

  node_handle_ = gazebo::transport::NodePtr(new transport::Node());
  node_handle_->Init();

  this->robot_namespace_ = "";
  if (!_sdf->HasElement("robotNamespace"))
  {
    ROS_INFO_NAMED("agribot_drive",
                   "AgribotDrive Plugin missing <robotNamespace>,"
                   " defaults to \"%s\"",
                   this->robot_namespace_.c_str());
  }
  else
  {
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }

  this->broadcast_tf_ = true;
  if (!_sdf->HasElement("broadcastTF"))
  {
    if (!this->broadcast_tf_)
      ROS_INFO_NAMED("agribot_drive",
                     "AgribotDrive Plugin (ns = %s) missing <broadcastTF>,"
                     " defaults to false.",
                     this->robot_namespace_.c_str());
    else
      ROS_INFO_NAMED("agribot_drive",
                     "AgribotDrive Plugin (ns = %s) missing <broadcastTF>,"
                     " defaults to true.",
                     this->robot_namespace_.c_str());
  }
  else
  {
    this->broadcast_tf_ = _sdf->GetElement("broadcastTF")->Get<bool>();
  }

  this->odometry_topic_ = "/agribot/odometry";
  if (!_sdf->HasElement("odometryTopic"))
  {
    ROS_WARN_NAMED("agribot_drive",
                   "AgribotDrive Plugin (ns = %s) missing <odometryTopic>,"
                   " defaults to \"%s\"",
                   this->robot_namespace_.c_str(), this->odometry_topic_.c_str());
  }
  else
  {
    this->odometry_topic_ = _sdf->GetElement("odometryTopic")->Get<std::string>();
  }

  this->encoder_topic_ = "/agribot/encoder/caterpillar";
  if (!_sdf->HasElement("encoderTopic"))
  {
    ROS_WARN_NAMED("agribot_drive",
                   "AgribotDrive Plugin (ns = %s) missing <encoderTopic>,"
                   " defaults to \"%s\"",
                   this->robot_namespace_.c_str(), this->encoder_topic_.c_str());
  }
  else
  {
    this->encoder_topic_ = _sdf->GetElement("encoderTopic")->Get<std::string>();
  }

  this->odometry_frame_ = "odom";
  if (!_sdf->HasElement("odometryFrame"))
  {
    ROS_WARN_NAMED("agribot_drive",
                   "AgribotDrive Plugin (ns = %s) missing <odometryFrame>,"
                   " defaults to \"%s\"",
                   this->robot_namespace_.c_str(), this->odometry_frame_.c_str());
  }
  else
  {
    this->odometry_frame_ = _sdf->GetElement("odometryFrame")->Get<std::string>();
  }

  this->robot_base_frame_ = "base_footprint";
  if (!_sdf->HasElement("robotBaseFrame"))
  {
    ROS_WARN_NAMED("agribot_drive",
                   "AgribotDrive Plugin (ns = %s) missing <robotBaseFrame>,"
                   " defaults to \"%s\"",
                   this->robot_namespace_.c_str(), this->robot_base_frame_.c_str());
  }
  else
  {
    this->robot_base_frame_ = _sdf->GetElement("robotBaseFrame")->Get<std::string>();
  }

  this->update_rate_ = 10.0;
  if (!_sdf->HasElement("updateRate"))
  {
    ROS_WARN_NAMED("agribot_drive",
                   "AgribotDrive Plugin (ns = %s) missing <updateRate>,"
                   " defaults to %f",
                   this->robot_namespace_.c_str(), this->update_rate_);
  }
  else
  {
    this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
  }

  this->wheel_separation_ = 0.22;
  if (!_sdf->HasElement("wheelSeparation"))
  {
    ROS_WARN_NAMED("agribot_drive",
                   "AgribotDrive Plugin (ns = %s) missing <wheelSeparation>,"
                   " defaults to %f",
                   this->robot_namespace_.c_str(), this->wheel_separation_);
  }
  else
  {
    this->wheel_separation_ = _sdf->GetElement("wheelSeparation")->Get<double>();
  }

  this->wheel_radius_ = 0.04;
  if (!_sdf->HasElement("wheelRadius"))
  {
    ROS_WARN_NAMED("agribot_drive",
                   "AgribotDrive Plugin (ns = %s) missing <wheelRadius>,"
                   " defaults to %f",
                   this->robot_namespace_.c_str(), this->wheel_radius_);
  }
  else
  {
    this->wheel_radius_ = _sdf->GetElement("wheelRadius")->Get<double>();
  }

  this->chasis_length_ = 1;
  if (!_sdf->HasElement("chasisLength"))
  {
    ROS_WARN_NAMED("agribot_drive",
                   "AgribotDrive Plugin (ns = %s) missing <chasisLength>,"
                   " defaults to %f",
                   this->robot_namespace_.c_str(), this->chasis_length_);
  }
  else
  {
    this->chasis_length_ = _sdf->GetElement("chasisLength")->Get<double>();
  }

  if (this->update_rate_ > 0.0)
  {
    this->update_period_ = 1.0 / this->update_rate_;
  }
  else
  {
    this->update_period_ = 0.0;
  }
  last_update_time_ = this->world->SimTime();

  alive_ = true;

  command_sub_topic_ = "/agribot/control/caterpillar";
  wheel_torque = 5.0;
  joints_.resize(joints_cnt);
  // Setup joint names:
  joints_[FRONT_LEFT_WHEEL].name = "front_left_wheel_joint";
  joints_[FRONT_RIGHT_WHEEL].name = "front_right_wheel_joint";
  joints_[BACK_LEFT_WHEEL].name = "back_left_wheel_joint";
  joints_[BACK_RIGHT_WHEEL].name = "back_right_wheel_joint";
  joints_[FRONT_CART].name = "front_cart_joint";
  joints_[BACK_CART].name = "back_cart_joint";
  joints_[FRONT_LOWER_HIT_ANG].name = "front_tpl_lower_hitches_revolute_joint";
  joints_[FRONT_UPPER_HIT_ANG].name = "front_tpl_upper_hitch_revolute_joint";
  joints_[FRONT_UPPER_HIT_HEI].name = "front_tpl_upper_hitch_prismatic_joint";
  joints_[BACK_LOWER_HIT_ANG].name = "back_tpl_lower_hitches_revolute_joint";
  joints_[BACK_UPPER_HIT_ANG].name = "back_tpl_upper_hitch_revolute_joint";
  joints_[BACK_UPPER_HIT_HEI].name = "back_tpl_upper_hitch_prismatic_joint";

  for (auto& joint : joints_)
  {
    joint.ptr = model_->GetJoint(joint.name);
    if (joint.ptr == nullptr)
    {
      ROS_WARN_NAMED("agribot_drive", "Couldn't find specified joint");
      gzthrow("[gazebo_ros_agribot] Couldn't find specified joint");
    }
    joint.ptr->SetForce(0, wheel_torque);
  }
  CreatePubsAndSubs();
  this->callback_queue_thread_ = boost::thread(boost::bind(&GazeboRosAgribot::QueueThread, this));
  // listen to the update event (broadcast every simulation iteration)
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosAgribot::onUpdate, this));
}

void GazeboRosAgribot::CreatePubsAndSubs()
{
  ros::SubscribeOptions so = ros::SubscribeOptions::create<agribot_msgs::AgribotCaterpillar>(
      command_sub_topic_, 1, boost::bind(&GazeboRosAgribot::cmdVelCallback, this, _1), ros::VoidPtr(), &queue_);
  ros::SubscribeOptions sub_cmd_tool_front = ros::SubscribeOptions::create<agribot_msgs::AgribotTool>(
      "agribot/control/tool/front", 1, boost::bind(&GazeboRosAgribot::cmdToolsFrontCallback, this, _1), ros::VoidPtr(),
      &queue_);
  ros::SubscribeOptions sub_cmd_tool_back = ros::SubscribeOptions::create<agribot_msgs::AgribotTool>(
      "agribot/control/tool/back", 1, boost::bind(&GazeboRosAgribot::cmdToolsBackCallback, this, _1), ros::VoidPtr(),
      &queue_);

  rosnode_ = new ros::NodeHandle(this->robot_namespace_);
  lamps_control_service_ = rosnode_->advertiseService("/agribot/hal_lights/change_lamps_configuration",
                                                      &GazeboRosAgribot::lampControlServiceCallback, this);
  cmd_vel_subscriber_ = rosnode_->subscribe(so);
  cmd_front_tool_subscriber_ = rosnode_->subscribe(sub_cmd_tool_front);
  cmd_back_tool_subscriber_ = rosnode_->subscribe(sub_cmd_tool_back);
  odometry_publisher_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
  encoder_publisher_ = rosnode_->advertise<agribot_msgs::AgribotCaterpillar>(encoder_topic_, 1);
  front_cart_publisher_ = rosnode_->advertise<std_msgs::Float32>("agribot/encoder/cart/front", 1);
  back_cart_publisher_ = rosnode_->advertise<std_msgs::Float32>("agribot/encoder/cart/back", 1);
  catterpilar_wheels_publisher_ =
      rosnode_->advertise<agribot_msgs::AgribotCaterpillarWheelsRotation>("agribot/encoder/caterpillar/rotations", 1);
  front_tool_publisher_ = rosnode_->advertise<agribot_msgs::AgribotTool>("agribot/encoder/tool/front", 1);
  back_tool_publisher_ = rosnode_->advertise<agribot_msgs::AgribotTool>("agribot/encoder/tool/back", 1);
  tf_prefix_ = tf::getPrefixParam(*rosnode_);
  transform_broadcaster_ = new tf::TransformBroadcaster();
  front_tpl_lower_hitches_revolute_command_pub_ = rosnode_->advertise<std_msgs::Float64>("/agribot/controller/position/"
                                                                                         "front_tpl_lower_hitches_"
                                                                                         "revolute_control/command",
                                                                                         1);
  front_tpl_upper_hitch_prismatic_command_pub_ = rosnode_->advertise<std_msgs::Float64>("/agribot/controller/position/"
                                                                                        "front_tpl_upper_hitch_"
                                                                                        "prismatic_control/command",
                                                                                        1);
  back_tpl_lower_hitches_revolute_command_pub_ = rosnode_->advertise<std_msgs::Float64>("/agribot/controller/"
                                                                                        "position/"
                                                                                        "back_tpl_lower_hitches_"
                                                                                        "revolute_control/command",
                                                                                        1);
  back_tpl_upper_hitch_prismatic_command_pub_ = rosnode_->advertise<std_msgs::Float64>("/agribot/controller/position/"
                                                                                       "back_tpl_upper_hitch_prismatic_"
                                                                                       "control/command",
                                                                                       1);
}

void GazeboRosAgribot::onUpdate()
{
  common::Time current_time = this->world->SimTime();
  double seconds_since_last_update = (current_time - last_update_time_).Double();
  if (seconds_since_last_update > update_period_)
  {
    publishOdometry(seconds_since_last_update);
    publishEncoder();
    publishCarts();
    publishTools();
    last_update_time_ = current_time;
  }
  updateVelocities();
  publishToolsControllerCommands();
}

void GazeboRosAgribot::updateJointPosition(int joint_index)
{
  joints_[joint_index].ptr->SetPosition(0, joints_[joint_index].position);
}

auto GazeboRosAgribot::getJointPosition(int joint_index) const
{
  return joints_[joint_index].ptr->Position(0);
}

void GazeboRosAgribot::updateJointVelocity(int joint_index)
{
  joints_[joint_index].ptr->SetVelocity(0, joints_[joint_index].speed / wheel_radius_);
}

auto GazeboRosAgribot::getJointVelocity(int joint_index) const
{
  return joints_[joint_index].ptr->GetVelocity(0);
}

void GazeboRosAgribot::updateVelocities()
{
#ifdef DEBUG_VEL
  double current_speed[4];
  current_speed[FRONT_LEFT_WHEEL] = getJointVelocity(FRONT_LEFT_WHEEL);
  current_speed[FRONT_RIGHT_WHEEL] = getJointVelocity(FRONT_RIGHT_WHEEL);
  current_speed[BACK_LEFT_WHEEL] = getJointVelocity(BACK_LEFT_WHEEL]);
  current_speed[BACK_RIGHT_WHEEL] = getJointVelocity(BACK_RIGHT_WHEEL);
#endif

  updateJointVelocity(FRONT_LEFT_WHEEL);
  updateJointVelocity(FRONT_RIGHT_WHEEL);
  updateJointVelocity(BACK_LEFT_WHEEL);
  updateJointVelocity(BACK_RIGHT_WHEEL);
#ifdef DEBUG_VEL
  ROS_DEBUG_NAMED("agribot_drive",
                  "Set  FL  speed = %lf, FR speed= %lf,"
                  " BL speed = %lf, BR speed= %lf",
                  joints_[FRONT_LEFT_WHEEL].speed, joints_[FRONT_RIGHT_WHEEL].speed, joints_[BACK_LEFT_WHEEL].speed,
                  joints_[BACK_RIGHT_WHEEL].speed);
  ROS_DEBUG_NAMED("agribot_drive",
                  "Current  speed = %lf, FR speed= %lf,"
                  " BL speed = %lf, BR speed= %lf",
                  current_speed[FRONT_LEFT_WHEEL], current_speed[FRONT_RIGHT_WHEEL], current_speed[BACK_LEFT_WHEEL],
                  current_speed[BACK_RIGHT_WHEEL]);
#endif
}

void GazeboRosAgribot::cmdVelCallback(const agribot_msgs::AgribotCaterpillar::ConstPtr& cmd_msg)
{
  boost::mutex::scoped_lock scoped_lock(lock);

  // TODO(abojda): Check equations.
  // v_r_p = ((2 * v_p) + (w_p * L)) / (2 * R) = (2*v+w*L/2)/R
  // v_l_p = ((2 * v_p) - (w_p * L)) / (2 * R) = (2*v-w*L/2)/R
  // v_r_p = ((2 * v_t) + (w_t * L)) / (2 * R) = (2*v+w*L/2)/R
  // v_l_p = ((2 * v_t) - (w_t * L)) / (2 * R) = (2*v-w*L/2)/R
  // v_t=sin(theta_1)/sin(theta_2)*v_p
  joints_[FRONT_RIGHT_WHEEL].speed = cmd_msg->front_linear + wheel_separation_ / 2.0 * cmd_msg->front_angular;
  joints_[FRONT_LEFT_WHEEL].speed = cmd_msg->front_linear - wheel_separation_ / 2.0 * cmd_msg->front_angular;
  joints_[BACK_RIGHT_WHEEL].speed = cmd_msg->back_linear + wheel_separation_ / 2.0 * cmd_msg->back_angular;
  joints_[BACK_LEFT_WHEEL].speed = cmd_msg->back_linear - wheel_separation_ / 2.0 * cmd_msg->back_angular;
}

void GazeboRosAgribot::cmdToolsFrontCallback(const agribot_msgs::AgribotTool::ConstPtr& tool_msg)
{
  front_cart_tool_setpoint_.red_valve_flow = tool_msg->red_valve_flow;
  front_cart_tool_setpoint_.green_valve_flow = tool_msg->green_valve_flow;
  front_cart_tool_setpoint_.blue_valve_flow = tool_msg->blue_valve_flow;
  front_cart_tool_setpoint_.pto_rpm = tool_msg->pto_rpm;
  front_cart_tool_setpoint_.tpl_height = tool_msg->tpl_height;
  front_cart_tool_setpoint_.tpl_angle = tool_msg->tpl_angle;
}

void GazeboRosAgribot::cmdToolsBackCallback(const agribot_msgs::AgribotTool::ConstPtr& tool_msg)
{
  back_cart_tool_setpoint_.red_valve_flow = tool_msg->red_valve_flow;
  back_cart_tool_setpoint_.green_valve_flow = tool_msg->green_valve_flow;
  back_cart_tool_setpoint_.blue_valve_flow = tool_msg->blue_valve_flow;
  back_cart_tool_setpoint_.pto_rpm = tool_msg->pto_rpm;
  back_cart_tool_setpoint_.tpl_height = tool_msg->tpl_height;
  back_cart_tool_setpoint_.tpl_angle = tool_msg->tpl_angle;
}

void GazeboRosAgribot::QueueThread()
{
  static const double timeout = 0.01;
  while (alive_ && rosnode_->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void GazeboRosAgribot::publishOdometry(double step_time)
{
  ros::Time current_time = ros::Time::now();
  std::string odom_frame = tf::resolve(tf_prefix_, odometry_frame_);
  std::string base_footprint_frame = tf::resolve(tf_prefix_, robot_base_frame_);
  // TODO(agribot) create some non-perfect odometry!
  // getting data for base_footprint to odom transform
  ignition::math::Pose3d pose = this->model_->WorldPose();
  tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
  tf::Vector3 vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
  tf::Transform base_footprint_to_odom(qt, vt);

  if (this->broadcast_tf_)
  {
    transform_broadcaster_->sendTransform(
        tf::StampedTransform(base_footprint_to_odom, current_time, odom_frame, base_footprint_frame));
    /*TODO Remove it when robot_localization package will transmit map->odom tf */
    tf::Quaternion qt_map(0, 0, 0, 1);
    tf::Vector3 vt_map(0, 0, 0);
    tf::Transform map_to_odom(qt_map, vt_map);
    transform_broadcaster_->sendTransform(tf::StampedTransform(map_to_odom, current_time, "map", odom_frame));
  }

  // publish odom topic
  odom_.pose.pose.position.x = pose.Pos().X();
  odom_.pose.pose.position.y = pose.Pos().Y();
  odom_.pose.pose.orientation.x = pose.Rot().X();
  odom_.pose.pose.orientation.y = pose.Rot().Y();
  odom_.pose.pose.orientation.z = pose.Rot().Z();
  odom_.pose.pose.orientation.w = pose.Rot().W();
  odom_.pose.covariance[0] = this->covariance_x_;
  odom_.pose.covariance[7] = this->covariance_y_;
  odom_.pose.covariance[14] = 1000000000000.0;
  odom_.pose.covariance[21] = 1000000000000.0;
  odom_.pose.covariance[28] = 1000000000000.0;
  odom_.pose.covariance[35] = this->covariance_yaw_;
  // get velocity in /odom frame
  ignition::math::Vector3d linear;
  linear = this->model_->WorldLinearVel();
  odom_.twist.twist.angular.z = this->model_->WorldAngularVel().Z();
  // convert velocity to child_frame_id (aka base_footprint)
  float yaw = pose.Rot().Yaw();
  odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
  odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
  odom_.twist.covariance[0] = this->covariance_x_;
  odom_.twist.covariance[7] = this->covariance_y_;
  odom_.twist.covariance[14] = 1000000000000.0;
  odom_.twist.covariance[21] = 1000000000000.0;
  odom_.twist.covariance[28] = 1000000000000.0;
  odom_.twist.covariance[35] = this->covariance_yaw_;
  odom_.header.stamp = current_time;
  odom_.header.frame_id = odom_frame;
  odom_.child_frame_id = base_footprint_frame;
  odometry_publisher_.publish(odom_);
}

void GazeboRosAgribot::publishEncoder()
{
  double current_speed[4];
  current_speed[FRONT_LEFT_WHEEL] = joints_[FRONT_LEFT_WHEEL].ptr->GetVelocity(0);
  current_speed[FRONT_RIGHT_WHEEL] = joints_[FRONT_RIGHT_WHEEL].ptr->GetVelocity(0);
  current_speed[BACK_LEFT_WHEEL] = joints_[BACK_LEFT_WHEEL].ptr->GetVelocity(0);
  current_speed[BACK_RIGHT_WHEEL] = joints_[BACK_RIGHT_WHEEL].ptr->GetVelocity(0);
  // w=(VR-VL)/d
  // v=(VR+VL)/2
  encoder_.header.stamp = ros::Time::now();
  encoder_.front_linear =
      (current_speed[FRONT_LEFT_WHEEL] * wheel_radius_ + current_speed[FRONT_RIGHT_WHEEL] * wheel_radius_) / 2;
  encoder_.front_angular =
      (current_speed[FRONT_RIGHT_WHEEL] * wheel_radius_ - current_speed[FRONT_LEFT_WHEEL] * wheel_radius_) /
      wheel_separation_;
  encoder_.back_linear =
      (current_speed[BACK_LEFT_WHEEL] * wheel_radius_ + current_speed[BACK_RIGHT_WHEEL] * wheel_radius_) / 2;
  encoder_.back_angular =
      (current_speed[BACK_RIGHT_WHEEL] * wheel_radius_ - current_speed[BACK_LEFT_WHEEL] * wheel_radius_) /
      wheel_separation_;
  encoder_publisher_.publish(encoder_);

  CaterpillarEncoderMsg_.header.stamp = ros::Time::now();
  CaterpillarEncoderMsg_.back_left = getJointPosition(BACK_LEFT_WHEEL);
  CaterpillarEncoderMsg_.back_right = getJointPosition(BACK_RIGHT_WHEEL);
  CaterpillarEncoderMsg_.front_left = getJointPosition(FRONT_LEFT_WHEEL);
  CaterpillarEncoderMsg_.front_right = getJointPosition(FRONT_RIGHT_WHEEL);
  catterpilar_wheels_publisher_.publish(CaterpillarEncoderMsg_);
}

void GazeboRosAgribot::publishCarts()
{
  FrontAngleMsg_.data = getJointPosition(FRONT_CART);
  BackAngleMsg_.data = getJointPosition(BACK_CART);
  front_cart_publisher_.publish(FrontAngleMsg_);
  back_cart_publisher_.publish(BackAngleMsg_);
}

void GazeboRosAgribot::publishTools()
{
  agribot_msgs::AgribotTool msg;
  msg.tpl_angle = getJointPosition(FRONT_LOWER_HIT_ANG);
  msg.tpl_height = getJointPosition(FRONT_UPPER_HIT_HEI);
  front_tool_publisher_.publish(msg);
  msg.tpl_angle = getJointPosition(BACK_LOWER_HIT_ANG);
  msg.tpl_height = getJointPosition(BACK_UPPER_HIT_HEI);
  back_tool_publisher_.publish(msg);
}

void GazeboRosAgribot::publishToolsControllerCommands() const
{
  std_msgs::Float64 msg;
  msg.data = front_cart_tool_setpoint_.tpl_angle;
  front_tpl_lower_hitches_revolute_command_pub_.publish(msg);
  msg.data = front_cart_tool_setpoint_.tpl_height;
  front_tpl_upper_hitch_prismatic_command_pub_.publish(msg);
  msg.data = back_cart_tool_setpoint_.tpl_angle;
  back_tpl_lower_hitches_revolute_command_pub_.publish(msg);
  msg.data = back_cart_tool_setpoint_.tpl_height;
  back_tpl_upper_hitch_prismatic_command_pub_.publish(msg);
}

bool GazeboRosAgribot::lampControlServiceCallback(
    agribot_msgs::LampsControl::Request& req,   // NOLINT(runtime/references)
    agribot_msgs::LampsControl::Response& res)  // NOLINT(runtime/references)
{
  return true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosAgribot)
}  // namespace gazebo
