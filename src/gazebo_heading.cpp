#include <algorithm>
#include <assert.h>
#include <agribot_simulation/gazebo_heading.h>
#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <math.h>
#include <sdf/sdf.hh>
#include <ros/ros.h>
#include <tf/tf.h>

//#define DEBUG_VEL
//#define DEBUG_TF
#define PI 3.14159265359

namespace gazebo
{

GazeboHeading::GazeboHeading() {}

// Destructor
GazeboHeading::~GazeboHeading() {}

// Load the controller
void GazeboHeading::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;
  this->world = _model->GetWorld();

  node_handle_ = gazebo::transport::NodePtr(new transport::Node());
  node_handle_->Init();

  this->robot_namespace_ = "";
  if (!_sdf->HasElement("robotNamespace"))
  {
    ROS_INFO_NAMED("agribot_drive", "Heading Plugin missing <robotNamespace>, defaults to \"%s\"",
                   this->robot_namespace_.c_str());
  }
  else
  {
    this->robot_namespace_ =
        _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }

  this->heading_topic_ = "/agribot/heading";
  if (!_sdf->HasElement("headingTopic"))
  {
    ROS_WARN_NAMED("agribot_drive", "Heading Plugin (ns = %s) missing <headingTopic>, defaults to \"%s\"",
                   this->robot_namespace_.c_str(), this->heading_topic_.c_str());
  }
  else
  {
    this->heading_topic_ = _sdf->GetElement("headingTopic")->Get<std::string>();
  }

  this->update_rate_ = 10000.0;
  if (!_sdf->HasElement("updateRate"))
  {
    ROS_WARN_NAMED("agribot_drive", "Heading Plugin (ns = %s) missing <updateRate>, defaults to %f",
                   this->robot_namespace_.c_str(), this->update_rate_);
  }
  else
  {
    this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
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

//   const std::string & abc ="base_footprint";

  this->link_name = "base_footprint";
  if (!_sdf->HasElement("headingLink"))
  {
    ROS_WARN_NAMED("agribot_drive", "Heading Plugin (ns = %s) missing <headingLink>, defaults to \"%s\"",
                   this->robot_namespace_.c_str(), this->link_name.c_str());
  }
  else
  {
    this->link_name = _sdf->GetElement("headingLink")->Get<std::string>();
  }
  link_ = this->model_->GetLink("base_footprint"); // TODO find out why  this->link_name doesn't work
  if (link_ == nullptr)
    {
      ROS_WARN_NAMED("agribot_drive", "Couldn't find specified link %s",this->link_name.c_str());
      gzthrow("[gazebo_ros_agribot] Couldn't find specified link");
    }
  CreatePubsAndSubs();
  this->callback_queue_thread_ =
      boost::thread(boost::bind(&GazeboHeading::QueueThread, this));
  // listen to the update event (broadcast every simulation iteration)
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboHeading::onUpdate, this));
}

void GazeboHeading::CreatePubsAndSubs()
{
  rosnode_ = new ros::NodeHandle(this->robot_namespace_);
  heading_publisher_ = rosnode_->advertise<sensor_msgs::Imu>(heading_topic_, 1);
}

void GazeboHeading::onUpdate()
{
  common::Time current_time = this->world->SimTime();
  double seconds_since_last_update = (current_time - last_update_time_).Double();
  if (seconds_since_last_update > update_period_)
  {
    publishHeading();
  }
  last_update_time_ += common::Time(update_period_);
}

void GazeboHeading::QueueThread()
{
  static const double timeout = 0.01;
  while (alive_ && rosnode_->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void GazeboHeading::publishHeading()
{
    ignition::math::Pose3d pose = this->link_->WorldPose();
    heading_.header.stamp = ros::Time::now();
    heading_.header.frame_id = "heading";
    heading_.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,pose.Pos().Z());  
    heading_publisher_.publish(heading_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboHeading)
} // namespace gazebo
