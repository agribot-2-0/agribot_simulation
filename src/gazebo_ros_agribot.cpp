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

    /*TODO: Get this data from urdf */
    command_sub_topic_ = "/agribot/front";
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

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ = event::Events::ConnectWorldUpdateBegin (
              std::bind ( &GazeboRosAgribot::onUpdate, this ) );
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
  // ============================================ //
  // = CONTROL COMMAND MSG SETUP (ROS->GAZEBO) = //
  // ============================================ //
  //
  // cmd_vel_subscriber_ = node_handle_->Subscribe(command_sub_topic_, &GazeboRosAgribot::cmdVelCallback, this);
  //
  // connect_ros_to_gazebo_topic_msg.set_ros_topic(command_sub_topic_);
  // connect_ros_to_gazebo_topic_msg.set_gazebo_topic(command_sub_topic_);
  // connect_ros_to_gazebo_topic_msg.set_msgtype(gz_std_msgs::ConnectRosToGazeboTopic::COMMAND_MOTOR_SPEED);
  // gz_connect_ros_to_gazebo_topic_pub->Publish(
  //     connect_ros_to_gazebo_topic_msg, true);
}

void GazeboRosAgribot::cmdVelCallback ( const agribot_msgs::AgribotCaterpillar::ConstPtr& cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    ROS_INFO_NAMED("agribot", "received cmd");
    // wheel_speed_[FRONT_LEFT] = cmd_msg->left;
    // wheel_speed_[FRONT_RIGHT] = cmd_msg->right;
}



GZ_REGISTER_MODEL_PLUGIN ( GazeboRosAgribot )
}
