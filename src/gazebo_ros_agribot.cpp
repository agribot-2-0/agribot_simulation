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

// #define DEBUG

namespace gazebo
{

enum {
    FRONT_RIGHT,
    FRONT_LEFT,
    BACK_RIGHT,
    BACK_LEFT,
    FRONT_CART,
    BACK_CART
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
    	  ROS_INFO_NAMED("agribot_drive", "AgribotDrive Plugin (ns = %s) missing <broadcastTF>, defaults to false.",this->robot_namespace_.c_str());
      else ROS_INFO_NAMED("agribot_drive", "AgribotDrive Plugin (ns = %s) missing <broadcastTF>, defaults to true.",this->robot_namespace_.c_str());

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
    } else 
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

    this->update_rate_ = 10000.0;
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
    } else 
    {
      this->wheel_radius_ = _sdf->GetElement("wheelRadius")->Get<double>();
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

    /*TODO: Get this data from urdf */
    command_sub_topic_ = "/agribot/control/caterpillar";
    wheel_torque = 5.0;
    joints_name_.resize ( 6 );
    joints_name_[FRONT_LEFT] = "front_left_wheel_joint";
    joints_name_[FRONT_RIGHT] = "front_right_wheel_joint";
    joints_name_[BACK_LEFT] = "back_left_wheel_joint";
    joints_name_[BACK_RIGHT] = "back_right_wheel_joint";
    joints_name_[FRONT_CART] = "front_cart_joint";
    joints_name_[BACK_CART] = "back_cart_joint";

    // Initialize joints
    joints_.resize ( 6 );
    joints_[FRONT_LEFT] = model_->GetJoint ( joints_name_[FRONT_LEFT] );
    joints_[FRONT_RIGHT] = model_->GetJoint ( joints_name_[FRONT_RIGHT] );
    joints_[FRONT_CART] = model_->GetJoint (  joints_name_[FRONT_CART] );
    joints_[BACK_LEFT] = model_->GetJoint (  joints_name_[BACK_LEFT] );
    joints_[BACK_RIGHT] = model_->GetJoint (  joints_name_[BACK_RIGHT] );
    joints_[BACK_CART] = model_->GetJoint (  joints_name_[BACK_CART] );

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

    for(auto cart_speed: cart_speed_)
    {
      cart_speed = 0.0;
    }

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
    encoder_publisher_ = rosnode_->advertise<agribot_msgs::AgribotCaterpillar>(encoder_topic_, 1);
    front_cart_publisher_ = rosnode_->advertise<std_msgs::Float32>("agribot/encoder/cart/front", 1);
    back_cart_publisher_ = rosnode_->advertise<std_msgs::Float32>("agribot/encoder/cart/back", 1);
    tf_prefix_ = tf::getPrefixParam(*rosnode_);
    transform_broadcaster_ = new tf::TransformBroadcaster();
}

// Update the controller
void GazeboRosAgribot::onUpdate()
{
  common::Time current_time = this->world->SimTime();
  double seconds_since_last_update = (current_time - last_update_time_).Double();
  if (seconds_since_last_update > update_period_) 
  {
    publishOdometry(seconds_since_last_update);
    publishEncoder();
    publishCarts();
  }
  UpdateVelocities();
  last_update_time_+= common::Time(update_period_);
}

void GazeboRosAgribot::UpdateVelocities()
{
  //v_r_p = ((2 * v_p) + (w_p * L)) / (2 * R) = (2*v+w*L/2)/R
  //v_l_p = ((2 * v_p) - (w_p * L)) / (2 * R) = (2*v-w*L/2)/R
  //v_r_p = ((2 * v_t) + (w_t * L)) / (2 * R) = (2*v+w*L/2)/R
  //v_l_p = ((2 * v_t) - (w_t * L)) / (2 * R) = (2*v-w*L/2)/R
  //v_t=sin(theta_1)/sin(theta_2)*v_p
  #ifdef DEBUG
  double current_speed[4];
  current_speed[FRONT_LEFT] = joints_[FRONT_LEFT]->GetVelocity (0);
  current_speed[FRONT_RIGHT] = joints_[FRONT_RIGHT]->GetVelocity (0);
  current_speed[BACK_LEFT] = joints_[BACK_LEFT]->GetVelocity (0);
  current_speed[BACK_RIGHT] = joints_[BACK_RIGHT]->GetVelocity (0);
  #endif
  cart_angle_[FRONT_CART] = joints_[FRONT_CART]->Position (0)+ignition::math::Angle::HalfPi.Radian();
  cart_angle_[BACK_CART] = joints_[BACK_CART]->Position (0)+ignition::math::Angle::HalfPi.Radian();
  cart_speed_[BACK_LINEAR]=sin(cart_angle_[FRONT_CART])/sin(cart_angle_[BACK_CART])*cart_speed_[FRONT_LINEAR];  
  wheel_speed_[FRONT_LEFT] = cart_speed_[FRONT_LINEAR] - cart_speed_[FRONT_ANGULAR] * wheel_separation_ / 2.0;
  wheel_speed_[FRONT_RIGHT] = cart_speed_[FRONT_LINEAR] + cart_speed_[FRONT_ANGULAR] * wheel_separation_ / 2.0;
  wheel_speed_[BACK_LEFT] = cart_speed_[BACK_LINEAR] - cart_speed_[BACK_ANGULAR] * wheel_separation_ / 2.0;
  wheel_speed_[BACK_RIGHT] = cart_speed_[BACK_LINEAR] + cart_speed_[BACK_ANGULAR] * wheel_separation_ / 2.0;
  joints_[FRONT_LEFT]->SetVelocity (0, wheel_speed_[FRONT_LEFT]/wheel_radius_);
  joints_[FRONT_RIGHT]->SetVelocity (0, wheel_speed_[FRONT_RIGHT]/wheel_radius_);
  joints_[BACK_LEFT]->SetVelocity (0, wheel_speed_[BACK_LEFT]/wheel_radius_);
  joints_[BACK_RIGHT]->SetVelocity (0, wheel_speed_[BACK_RIGHT]/wheel_radius_);
  #ifdef DEBUG
  ROS_INFO_NAMED("agribot_drive", "Read FL speed = %lf, FR speed= %lf, BL speed = %lf, BR speed= %lf", 
                  current_speed[FRONT_LEFT], current_speed[FRONT_RIGHT],
                  current_speed[BACK_LEFT] , current_speed[BACK_RIGHT]);

  ROS_INFO_NAMED("agribot_drive", "Set  FL speed = %lf, FR speed= %lf, BL speed = %lf, BR speed= %lf", 
                  wheel_speed_[FRONT_LEFT]/wheel_radius_, wheel_speed_[FRONT_RIGHT]/wheel_radius_,
                  wheel_speed_[BACK_LEFT]/wheel_radius_, wheel_speed_[BACK_RIGHT]/wheel_radius_);
  #endif
}

void GazeboRosAgribot::cmdVelCallback ( const agribot_msgs::AgribotCaterpillar::ConstPtr& cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    cart_speed_[FRONT_LINEAR] = cmd_msg->front_linear;
    cart_speed_[FRONT_ANGULAR] = cmd_msg->front_angular;
    //cart_speed_[BACK_LINEAR] = cmd_msg->back_linear;  We need 3 controls, not 4. This one will be computed.
    cart_speed_[BACK_ANGULAR] = cmd_msg->back_angular;
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
      publishWheelTF();
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

  void GazeboRosAgribot::publishWheelTF()
  {
    ros::Time current_time = ros::Time::now();
    for(auto joint: joints_)
    {
        std::string wheel_frame = joint->GetChild()->GetName();
        std::string wheel_parent_frame = joint->GetParent()->GetName();

    #if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d poseWheel = joint->GetChild()->RelativePose();
    #else
        ignition::math::Pose3d poseWheel = joint->GetChild()->GetRelativePose().Ign();
    #endif

        tf::Quaternion qt ( poseWheel.Rot().X(), poseWheel.Rot().Y(), poseWheel.Rot().Z(), poseWheel.Rot().W() );
        tf::Vector3 vt ( poseWheel.Pos().X(), poseWheel.Pos().Y(), poseWheel.Pos().Z() );

        tf::Transform tfWheel ( qt, vt );
        transform_broadcaster_->sendTransform (
            tf::StampedTransform ( tfWheel, current_time, wheel_parent_frame, wheel_frame ) );
    }
  }

  void GazeboRosAgribot::publishEncoder()
  {
    double current_speed[4];
    current_speed[FRONT_LEFT] = joints_[FRONT_LEFT]->GetVelocity (0);
    current_speed[FRONT_RIGHT] = joints_[FRONT_RIGHT]->GetVelocity (0);
    current_speed[BACK_LEFT] = joints_[BACK_LEFT]->GetVelocity (0);
    current_speed[BACK_RIGHT] = joints_[BACK_RIGHT]->GetVelocity (0);
    //w=(VR-VL)/d
    //v=(VR+VL)/2
    encoder_.front_linear=(current_speed[FRONT_LEFT]*wheel_radius_+current_speed[FRONT_RIGHT]*wheel_radius_)/2;
    encoder_.front_angular=(current_speed[FRONT_LEFT]*wheel_radius_-current_speed[FRONT_RIGHT]*wheel_radius_)/wheel_separation_;
    encoder_.back_linear=(current_speed[BACK_LEFT]*wheel_radius_+current_speed[BACK_RIGHT]*wheel_radius_)/2;
    encoder_.back_angular=(current_speed[BACK_LEFT]*wheel_radius_-current_speed[BACK_RIGHT]*wheel_radius_)/wheel_separation_;
    encoder_publisher_.publish(encoder_);
  }

  void GazeboRosAgribot::publishCarts()
  {
    FrontAngleMsg_.data=joints_[FRONT_CART]->GetPose(0);
    BackAngleMsg_.data=joints_[BACK_CART]->GetPose(0);
    front_cart_publisher_.publish(FrontAngleMsg_);
    back_cart_publisher_.publish(BackAngleMsg_);
  }

GZ_REGISTER_MODEL_PLUGIN ( GazeboRosAgribot )
}