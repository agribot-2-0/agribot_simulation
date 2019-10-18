#include <algorithm>
#include <assert.h>

#include <agribot_simulation/gazebo_ros_agribot.h>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>

namespace gazebo
{

enum {
    FRONT_RIGHT,
    FRONT_LEFT,
    BACK_RIGHT,
    BACK_LEFT
};

enum {
    FRONT_LINEAR,
    FRONT_ANGULAR,
    BACK_LINEAR,
    BACK_ANGULAR
};

GazeboRosAgribot::GazeboRosAgribot() {}

// Destructor
GazeboRosAgribot::~GazeboRosAgribot() {}

// Load the controller
void GazeboRosAgribot::Load ( physics::ModelPtr _model, sdf::ElementPtr _sdf )
{
    model_ = _model;
    this->world = _model->GetWorld();

    node_handle_ = gazebo::transport::NodePtr(new transport::Node());
    // Initialise with default namespace (typically /gazebo/default/)
    node_handle_->Init();

    this->robot_namespace_ = "";
    if (!_sdf->HasElement("robotNamespace")) {
      ROS_INFO_NAMED("agribot_drive", "AgribotDrive Plugin missing <robotNamespace>, defaults to \"%s\"",
          this->robot_namespace_.c_str());
    } else {
      this->robot_namespace_ =
        _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }

<<<<<<< HEAD
    this->broadcast_tf_ = true;
    if (!_sdf->HasElement("broadcastTF")) {
      if (!this->broadcast_tf_)
    	  ROS_INFO_NAMED("agribot_drive", "AgribotDrive Plugin (ns = %s) missing <broadcastTF>, defaults to false.",this->robot_namespace_.c_str());
      else ROS_INFO_NAMED("agribot_drive", "AgribotDrive Plugin (ns = %s) missing <broadcastTF>, defaults to true.",this->robot_namespace_.c_str());

    } else {
      this->broadcast_tf_ = _sdf->GetElement("broadcastTF")->Get<bool>();
    }

    this->odometry_topic_ = "/agribot/encoder";
    if (!_sdf->HasElement("odometryTopic")) {
      ROS_WARN_NAMED("agribot_drive", "AgribotDrive Plugin (ns = %s) missing <odometryTopic>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->odometry_topic_.c_str());
    } else {
      this->odometry_topic_ = _sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    this->odometry_frame_ = "/agribot/encoder";
    if (!_sdf->HasElement("odometryFrame")) {
      ROS_WARN_NAMED("agribot_drive", "AgribotDrive Plugin (ns = %s) missing <odometryFrame>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->odometry_frame_.c_str());
    } else {
      this->odometry_frame_ = _sdf->GetElement("odometryFrame")->Get<std::string>();
    }

    this->robot_base_frame_ = "base_footprint";
    if (!_sdf->HasElement("robotBaseFrame")) {
      ROS_WARN_NAMED("agribot_drive", "AgribotDrive Plugin (ns = %s) missing <robotBaseFrame>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->robot_base_frame_.c_str());
    } else {
      this->robot_base_frame_ = _sdf->GetElement("robotBaseFrame")->Get<std::string>();
    }

    this->update_rate_ = 10000.0;
    if (!_sdf->HasElement("updateRate")) {
      ROS_WARN_NAMED("agribot_drive", "AgribotDrive Plugin (ns = %s) missing <updateRate>, defaults to %f",
          this->robot_namespace_.c_str(), this->update_rate_);
    } else {
      this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
    }

     this->wheel_separation_ = 0.22;
    if (!_sdf->HasElement("wheelSeparation")) {
      ROS_WARN_NAMED("agribot_drive", "AgribotDrive Plugin (ns = %s) missing <wheelSeparation>, defaults to %f",
          this->robot_namespace_.c_str(), this->wheel_separation_);
    } else {
      this->wheel_separation_ = _sdf->GetElement("wheelSeparation")->Get<double>();
    }

     if (this->update_rate_ > 0.0) {
      this->update_period_ = 1.0 / this->update_rate_;
    } else {
      this->update_period_ = 0.0;
    }
    last_update_time_ = this->world->SimTime();

=======
>>>>>>> 687938f2695dad50791ff6566ba0441afc07ec54
    alive_ = true;

   


    /*TODO: Get this data from urdf */
    command_sub_topic_ = "/agribot/cmdvel";
    wheel_torque = 5.0;
    joints_name_.resize ( 4 );
    joints_name_[FRONT_LEFT] = "front_left_wheel_joint";
    joints_name_[FRONT_RIGHT] = "front_right_wheel_joint";
    joints_name_[BACK_LEFT] = "back_left_wheel_joint";
    joints_name_[BACK_RIGHT] = "back_right_wheel_joint";

    // Initialize joints
    joints_.resize ( 4 );
    joints_[FRONT_LEFT] = model_->GetJoint ( joints_name_[FRONT_LEFT] );
    joints_[FRONT_RIGHT] = model_->GetJoint ( joints_name_[FRONT_RIGHT] );
    joints_[BACK_LEFT] = model_->GetJoint (  joints_name_[BACK_LEFT] );
    joints_[BACK_RIGHT] = model_->GetJoint (  joints_name_[BACK_RIGHT] );

    for(auto joint: joints_)
    {
      if (joint == nullptr)
      {
        gzthrow("[gazebo_ros_agribot] Couldn't find specified joint");
      }

      joint->SetForce(0, wheel_torque);
    }

    // Initialize velocity stuff
    for(auto wheel_speed: wheel_speed_)
    {
      wheel_speed = 0.0;
    }

<<<<<<< HEAD
    for(auto cart_speed: cart_speed_)
    {
      cart_speed = 0.0;
    }
    
=======
>>>>>>> 687938f2695dad50791ff6566ba0441afc07ec54
    CreatePubsAndSubs();

    

    this->callback_queue_thread_ =
      boost::thread(boost::bind(&GazeboRosAgribot::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ = event::Events::ConnectWorldUpdateBegin (
              boost::bind ( &GazeboRosAgribot::onUpdate, this ) );
}


void GazeboRosAgribot::CreatePubsAndSubs()
{
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<agribot_msgs::AgribotCaterpillar>(command_sub_topic_, 1,
          boost::bind(&GazeboRosAgribot::cmdVelCallback, this, _1),
          ros::VoidPtr(), &queue_);
    
    rosnode_ = new ros::NodeHandle(this->robot_namespace_);
    cmd_vel_subscriber_ = rosnode_->subscribe(so);
    odometry_publisher_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);

    tf_prefix_ = tf::getPrefixParam(*rosnode_);
    transform_broadcaster_ = new tf::TransformBroadcaster();
}

// Update the controller
void GazeboRosAgribot::onUpdate()
{
  common::Time current_time = this->world->SimTime();
  double seconds_since_last_update = (current_time - last_update_time_).Double();
  if (seconds_since_last_update > update_period_) {
    publishOdometry(seconds_since_last_update);
  }
  UpdateVelocities();
<<<<<<< HEAD
  last_update_time_+= common::Time(update_period_);
=======
>>>>>>> 687938f2695dad50791ff6566ba0441afc07ec54
}

void GazeboRosAgribot::UpdateVelocities()
{
  wheel_speed_[FRONT_LEFT] = cart_speed_[FRONT_LINEAR] + cart_speed_[FRONT_ANGULAR] * wheel_separation_ / 2.0;
  wheel_speed_[FRONT_RIGHT] = cart_speed_[FRONT_LINEAR] - cart_speed_[FRONT_ANGULAR] * wheel_separation_ / 2.0;
  wheel_speed_[BACK_LEFT] = cart_speed_[BACK_LINEAR] + cart_speed_[BACK_ANGULAR] * wheel_separation_ / 2.0;
  wheel_speed_[BACK_RIGHT] = cart_speed_[BACK_LINEAR] + cart_speed_[BACK_ANGULAR] * wheel_separation_ / 2.0;

  joints_[FRONT_LEFT]->SetVelocity (0, wheel_speed_[FRONT_LEFT]);
  joints_[FRONT_RIGHT]->SetVelocity (0, wheel_speed_[FRONT_RIGHT]);
  joints_[BACK_LEFT]->SetVelocity (0, wheel_speed_[BACK_LEFT]);
  joints_[BACK_RIGHT]->SetVelocity (0, wheel_speed_[BACK_RIGHT]);
}

<<<<<<< HEAD
void GazeboRosAgribot::cmdVelCallback ( const agribot_msgs::AgribotCaterpillar::ConstPtr& cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    ROS_INFO_NAMED("agribot", "received vel cmd");
    cart_speed_[FRONT_LINEAR] = cmd_msg->front_linear;
    cart_speed_[FRONT_ANGULAR] = cmd_msg->front_angular;
    cart_speed_[BACK_LINEAR] = cmd_msg->back_linear;
    cart_speed_[BACK_ANGULAR] = cmd_msg->back_angular;
}

=======
void GazeboRosAgribot::CreatePubsAndSubs()
{
  ros::SubscribeOptions so_front =
      ros::SubscribeOptions::create<agribot_msgs::AgribotCaterpillar>(command_front_sub_topic_, 1,
          boost::bind(&GazeboRosAgribot::cmdFrontVelCallback, this, _1),
          ros::VoidPtr(), &queue_);
    ros::SubscribeOptions so_back =
      ros::SubscribeOptions::create<agribot_msgs::AgribotCaterpillar>(command_back_sub_topic_, 1,
          boost::bind(&GazeboRosAgribot::cmdBackVelCallback, this, _1),
          ros::VoidPtr(), &queue_);

    rosnode_ = new ros::NodeHandle(this->robot_namespace_);
    cmd_front_vel_subscriber_ = rosnode_->subscribe(so_front);
    cmd_back_vel_subscriber_ = rosnode_->subscribe(so_back);
}

void GazeboRosAgribot::cmdFrontVelCallback ( const agribot_msgs::AgribotCaterpillar::ConstPtr& cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    wheel_speed_[FRONT_LEFT] = cmd_msg->left;
    wheel_speed_[FRONT_RIGHT] = cmd_msg->right;
}

void GazeboRosAgribot::cmdBackVelCallback ( const agribot_msgs::AgribotCaterpillar::ConstPtr& cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    wheel_speed_[BACK_LEFT] = cmd_msg->left;
    wheel_speed_[BACK_RIGHT] = cmd_msg->right;
}
>>>>>>> 687938f2695dad50791ff6566ba0441afc07ec54

void GazeboRosAgribot::QueueThread()
{
    static const double timeout = 0.01;

    while (alive_ && rosnode_->ok())
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  void GazeboRosAgribot::publishOdometry(double step_time) {
    
    ros::Time current_time = ros::Time::now();
    std::string odom_frame = tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_footprint_frame = tf::resolve(tf_prefix_, robot_base_frame_);

    // TODO create some non-perfect odometry!
    // getting data for base_footprint to odom transform

    ignition::math::Pose3d pose = this->model_->WorldPose();

    tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
    tf::Vector3 vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

    tf::Transform base_footprint_to_odom(qt, vt);
    if (this->broadcast_tf_) {
    	transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom, current_time, odom_frame, base_footprint_frame));
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


GZ_REGISTER_MODEL_PLUGIN ( GazeboRosAgribot )
}
