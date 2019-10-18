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

GazeboRosAgribot::GazeboRosAgribot() {}

// Destructor
GazeboRosAgribot::~GazeboRosAgribot() {}

// Load the controller
void GazeboRosAgribot::Load ( physics::ModelPtr _model, sdf::ElementPtr _sdf )
{
    model_ = _model;

    node_handle_ = gazebo::transport::NodePtr(new transport::Node());
    // Initialise with default namespace (typically /gazebo/default/)
    node_handle_->Init();

    this->robot_namespace_ = "";
    if (!_sdf->HasElement("robotNamespace")) {
      ROS_INFO_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin missing <robotNamespace>, defaults to \"%s\"",
          this->robot_namespace_.c_str());
    } else {
      this->robot_namespace_ =
        _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }

    alive_ = true;

    /*TODO: Get this data from urdf */
    command_front_sub_topic_ = "/agribot/front";
    command_back_sub_topic_ = "/agribot/back";
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

    CreatePubsAndSubs();

    this->callback_queue_thread_ =
      boost::thread(boost::bind(&GazeboRosAgribot::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ = event::Events::ConnectWorldUpdateBegin (
              boost::bind ( &GazeboRosAgribot::onUpdate, this ) );
}

// Update the controller
void GazeboRosAgribot::onUpdate()
{
  UpdateVelocities();
}

void GazeboRosAgribot::UpdateVelocities()
{
  joints_[FRONT_LEFT]->SetVelocity (0, wheel_speed_[FRONT_LEFT]);
  joints_[FRONT_RIGHT]->SetVelocity (0, wheel_speed_[FRONT_RIGHT]);
  joints_[BACK_LEFT]->SetVelocity (0, wheel_speed_[BACK_LEFT]);
  joints_[BACK_RIGHT]->SetVelocity (0, wheel_speed_[BACK_RIGHT]);
}

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

void GazeboRosAgribot::QueueThread()
{
    static const double timeout = 0.01;

    while (alive_ && rosnode_->ok())
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

GZ_REGISTER_MODEL_PLUGIN ( GazeboRosAgribot )
}
