#include <algorithm>
#include <assert.h>
#include <agribot_simulation/gazebo_ros_agribot.h>
#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <math.h>
#include <sdf/sdf.hh>
#include <ros/ros.h>

//#define DEBUG_VEL
//#define DEBUG_TF
#define PI 3.14159265359

namespace gazebo
{

enum
{
  FRONT_RIGHT,
  FRONT_LEFT,
  BACK_RIGHT,
  BACK_LEFT,
  FRONT_CART,
  BACK_CART
};
enum
{
  FRONT_LINEAR,
  FRONT_ANGULAR,
  BACK_LINEAR,
  BACK_ANGULAR
};

GazeboRosAgribot::GazeboRosAgribot() {}

// Destructor
GazeboRosAgribot::~GazeboRosAgribot() {}

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
    ROS_INFO_NAMED("agribot_drive", "AgribotDrive Plugin missing <robotNamespace>, defaults to \"%s\"",
                   this->robot_namespace_.c_str());
  }
  else
  {
    this->robot_namespace_ =
        _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }

  this->broadcast_tf_ = true;
  if (!_sdf->HasElement("broadcastTF"))
  {
    if (!this->broadcast_tf_)
      ROS_INFO_NAMED("agribot_drive", "AgribotDrive Plugin (ns = %s) missing <broadcastTF>, defaults to false.", this->robot_namespace_.c_str());
    else
      ROS_INFO_NAMED("agribot_drive", "AgribotDrive Plugin (ns = %s) missing <broadcastTF>, defaults to true.", this->robot_namespace_.c_str());
  }
  else
  {
    this->broadcast_tf_ = _sdf->GetElement("broadcastTF")->Get<bool>();
  }

  this->odometry_topic_ = "/agribot/odometry";
  if (!_sdf->HasElement("odometryTopic"))
  {
    ROS_WARN_NAMED("agribot_drive", "AgribotDrive Plugin (ns = %s) missing <odometryTopic>, defaults to \"%s\"",
                   this->robot_namespace_.c_str(), this->odometry_topic_.c_str());
  }
  else
  {
    this->odometry_topic_ = _sdf->GetElement("odometryTopic")->Get<std::string>();
  }

  this->encoder_topic_ = "/agribot/encoder/caterpillar";
  if (!_sdf->HasElement("encoderTopic"))
  {
    ROS_WARN_NAMED("agribot_drive", "AgribotDrive Plugin (ns = %s) missing <encoderTopic>, defaults to \"%s\"",
                   this->robot_namespace_.c_str(), this->encoder_topic_.c_str());
  }
  else
  {
    this->encoder_topic_ = _sdf->GetElement("encoderTopic")->Get<std::string>();
  }

  this->odometry_frame_ = "odom";
  if (!_sdf->HasElement("odometryFrame"))
  {
    ROS_WARN_NAMED("agribot_drive", "AgribotDrive Plugin (ns = %s) missing <odometryFrame>, defaults to \"%s\"",
                   this->robot_namespace_.c_str(), this->odometry_frame_.c_str());
  }
  else
  {
    this->odometry_frame_ = _sdf->GetElement("odometryFrame")->Get<std::string>();
  }

  this->robot_base_frame_ = "base_footprint";
  if (!_sdf->HasElement("robotBaseFrame"))
  {
    ROS_WARN_NAMED("agribot_drive", "AgribotDrive Plugin (ns = %s) missing <robotBaseFrame>, defaults to \"%s\"",
                   this->robot_namespace_.c_str(), this->robot_base_frame_.c_str());
  }
  else
  {
    this->robot_base_frame_ = _sdf->GetElement("robotBaseFrame")->Get<std::string>();
  }

  this->update_rate_ = 10.0;
  if (!_sdf->HasElement("updateRate"))
  {
    ROS_WARN_NAMED("agribot_drive", "AgribotDrive Plugin (ns = %s) missing <updateRate>, defaults to %f",
                   this->robot_namespace_.c_str(), this->update_rate_);
  }
  else
  {
    this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
  }

  this->wheel_separation_ = 0.22;
  if (!_sdf->HasElement("wheelSeparation"))
  {
    ROS_WARN_NAMED("agribot_drive", "AgribotDrive Plugin (ns = %s) missing <wheelSeparation>, defaults to %f",
                   this->robot_namespace_.c_str(), this->wheel_separation_);
  }
  else
  {
    this->wheel_separation_ = _sdf->GetElement("wheelSeparation")->Get<double>();
  }

  this->wheel_radius_ = 0.04;
  if (!_sdf->HasElement("wheelRadius"))
  {
    ROS_WARN_NAMED("agribot_drive", "AgribotDrive Plugin (ns = %s) missing <wheelRadius>, defaults to %f",
                   this->robot_namespace_.c_str(), this->wheel_radius_);
  }
  else
  {
    this->wheel_radius_ = _sdf->GetElement("wheelRadius")->Get<double>();
  }

  this->chasis_length_ = 1;
  if (!_sdf->HasElement("chasisLength"))
  {
    ROS_WARN_NAMED("agribot_drive", "AgribotDrive Plugin (ns = %s) missing <chasisLength>, defaults to %f",
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
  joints_name_.resize(6);
  joints_name_[FRONT_LEFT] = "front_left_wheel_joint";
  joints_name_[FRONT_RIGHT] = "front_right_wheel_joint";
  joints_name_[BACK_LEFT] = "back_left_wheel_joint";
  joints_name_[BACK_RIGHT] = "back_right_wheel_joint";
  joints_name_[FRONT_CART] = "front_cart_joint";
  joints_name_[BACK_CART] = "back_cart_joint";
  // Initialize joints
  joints_.resize(6);
  joints_[FRONT_LEFT] = model_->GetJoint(joints_name_[FRONT_LEFT]);
  joints_[FRONT_RIGHT] = model_->GetJoint(joints_name_[FRONT_RIGHT]);
  joints_[FRONT_CART] = model_->GetJoint(joints_name_[FRONT_CART]);
  joints_[BACK_LEFT] = model_->GetJoint(joints_name_[BACK_LEFT]);
  joints_[BACK_RIGHT] = model_->GetJoint(joints_name_[BACK_RIGHT]);
  joints_[BACK_CART] = model_->GetJoint(joints_name_[BACK_CART]);
  for (auto joint : joints_)
  {
    if (joint == nullptr)
    {
      ROS_WARN_NAMED("agribot_drive", "Couldn't find specified joint");
      gzthrow("[gazebo_ros_agribot] Couldn't find specified joint");
    }

    joint->SetForce(0, wheel_torque);
  }
  // Initialize velocity stuff
  for (auto wheel_speed : wheel_speed_)
  {
    wheel_speed = 0.0;
  }

  for (auto cart_speed : cart_speed_)
  {
    cart_speed = 0.0;
  }
  CreatePubsAndSubs();
  this->callback_queue_thread_ =
      boost::thread(boost::bind(&GazeboRosAgribot::QueueThread, this));
  // listen to the update event (broadcast every simulation iteration)
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosAgribot::onUpdate, this));
}

void GazeboRosAgribot::CreatePubsAndSubs()
{
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<agribot_msgs::AgribotCaterpillar>
      (command_sub_topic_, 1, boost::bind(&GazeboRosAgribot::cmdVelCallback,
      this, _1), ros::VoidPtr(), &queue_);
  rosnode_ = new ros::NodeHandle(this->robot_namespace_);
  cmd_vel_subscriber_ = rosnode_->subscribe(so);
  odometry_publisher_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
  encoder_publisher_ = rosnode_->advertise<agribot_msgs::AgribotCaterpillar>(encoder_topic_, 1);
  front_cart_publisher_ = rosnode_->advertise<std_msgs::Float32>("agribot/encoder/cart/front", 1);
  back_cart_publisher_ = rosnode_->advertise<std_msgs::Float32>("agribot/encoder/cart/back", 1);
  tf_prefix_ = tf::getPrefixParam(*rosnode_);
  transform_broadcaster_ = new tf::TransformBroadcaster();
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
    last_update_time_ = current_time;
  }
  UpdateVelocities();
}

void GazeboRosAgribot::UpdateVelocities()
{
//v_r_p = ((2 * v_p) + (w_p * L)) / (2 * R) = (2*v+w*L/2)/R
//v_l_p = ((2 * v_p) - (w_p * L)) / (2 * R) = (2*v-w*L/2)/R
//v_r_p = ((2 * v_t) + (w_t * L)) / (2 * R) = (2*v+w*L/2)/R
//v_l_p = ((2 * v_t) - (w_t * L)) / (2 * R) = (2*v-w*L/2)/R
//v_t=sin(theta_1)/sin(theta_2)*v_p
#ifdef DEBUG_VEL
  double current_speed[4];
  current_speed[FRONT_LEFT] = joints_[FRONT_LEFT]->GetVelocity(0);
  current_speed[FRONT_RIGHT] = joints_[FRONT_RIGHT]->GetVelocity(0);
  current_speed[BACK_LEFT] = joints_[BACK_LEFT]->GetVelocity(0);
  current_speed[BACK_RIGHT] = joints_[BACK_RIGHT]->GetVelocity(0);
#endif
  wheel_speed_[FRONT_RIGHT] = cart_speed_[FRONT_LINEAR] + wheel_separation_ / 2.0 * cart_speed_[FRONT_ANGULAR];
  wheel_speed_[FRONT_LEFT] = cart_speed_[FRONT_LINEAR] - wheel_separation_ / 2.0 * cart_speed_[FRONT_ANGULAR];
  wheel_speed_[BACK_RIGHT] = cart_speed_[BACK_LINEAR] + wheel_separation_ / 2.0 * cart_speed_[BACK_ANGULAR];
  wheel_speed_[BACK_LEFT] = cart_speed_[BACK_LINEAR] - wheel_separation_ / 2.0 * cart_speed_[BACK_ANGULAR];

  joints_[FRONT_LEFT]->SetVelocity(0, wheel_speed_[FRONT_LEFT] / wheel_radius_);
  joints_[FRONT_RIGHT]->SetVelocity(0, wheel_speed_[FRONT_RIGHT] / wheel_radius_);
  joints_[BACK_LEFT]->SetVelocity(0, wheel_speed_[BACK_LEFT] / wheel_radius_);
  joints_[BACK_RIGHT]->SetVelocity(0, wheel_speed_[BACK_RIGHT] / wheel_radius_);
#ifdef DEBUG_VEL
  ROS_DEBUG_NAMED("agribot_drive", "Set  FL  speed = %lf, FR speed= %lf, BL speed = %lf, BR speed= %lf",
                  wheel_speed_[FRONT_LEFT], wheel_speed_[FRONT_RIGHT],
                  wheel_speed_[BACK_LEFT], wheel_speed_[BACK_RIGHT]);
  ROS_DEBUG_NAMED("agribot_drive", "Current  speed = %lf, FR speed= %lf, BL speed = %lf, BR speed= %lf",
                  current_speed[FRONT_LEFT], current_speed[FRONT_RIGHT],
                  current_speed[BACK_LEFT], current_speed[BACK_RIGHT]);
#endif
}

void GazeboRosAgribot::cmdVelCallback(const agribot_msgs::AgribotCaterpillar::ConstPtr &cmd_msg)
{
  boost::mutex::scoped_lock scoped_lock(lock);
  cart_speed_[FRONT_LINEAR] = cmd_msg->front_linear;
  cart_speed_[FRONT_ANGULAR] = cmd_msg->front_angular*PI/180;
  cart_speed_[BACK_LINEAR] = cmd_msg->back_linear;
  cart_speed_[BACK_ANGULAR] = cmd_msg->back_angular*PI/180;
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
  // TODO create some non-perfect odometry!
  // getting data for base_footprint to odom transform
  ignition::math::Pose3d pose = this->model_->WorldPose();
  tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
  tf::Vector3 vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
  tf::Transform base_footprint_to_odom(qt, vt);

  if (this->broadcast_tf_)
  {
    transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom, current_time, odom_frame, base_footprint_frame));
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
  current_speed[FRONT_LEFT] = joints_[FRONT_LEFT]->GetVelocity(0);
  current_speed[FRONT_RIGHT] = joints_[FRONT_RIGHT]->GetVelocity(0);
  current_speed[BACK_LEFT] = joints_[BACK_LEFT]->GetVelocity(0);
  current_speed[BACK_RIGHT] = joints_[BACK_RIGHT]->GetVelocity(0);
  //w=(VR-VL)/d
  //v=(VR+VL)/2
  encoder_.header.stamp = ros::Time::now();
  encoder_.header.frame_id = "control";
  encoder_.front_linear = (current_speed[FRONT_LEFT] * wheel_radius_ + current_speed[FRONT_RIGHT] * wheel_radius_) / 2;
  encoder_.front_angular = (current_speed[FRONT_LEFT] * wheel_radius_ - current_speed[FRONT_RIGHT] * wheel_radius_) / wheel_separation_;
  encoder_.back_linear = (current_speed[BACK_LEFT] * wheel_radius_ + current_speed[BACK_RIGHT] * wheel_radius_) / 2;
  encoder_.back_angular = (current_speed[BACK_LEFT] * wheel_radius_ - current_speed[BACK_RIGHT] * wheel_radius_) / wheel_separation_;
  encoder_publisher_.publish(encoder_);
}

void GazeboRosAgribot::publishCarts()
{
  FrontAngleMsg_.data = joints_[FRONT_CART]->Position(0)*180/PI;
  BackAngleMsg_.data = joints_[BACK_CART]->Position(0)*180/PI;
  front_cart_publisher_.publish(FrontAngleMsg_);
  back_cart_publisher_.publish(BackAngleMsg_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosAgribot)
} // namespace gazebo
