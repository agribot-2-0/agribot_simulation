//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <agribot_simulation/external/gazebo_inclinometer.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <tf/tf.h>

namespace gazebo {

GazeboInclinometer::GazeboInclinometer()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboInclinometer::~GazeboInclinometer()
{
  updateTimer.Disconnect(updateConnection);

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboInclinometer::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world = _model->GetWorld();

  // load parameters
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();
  else
    namespace_.clear();

  if (!_sdf->HasElement("topicName"))
    topic_ = "inclinometer";
  else
    topic_ = _sdf->GetElement("topicName")->Get<std::string>();


  if (_sdf->HasElement("bodyName"))
  {
    link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
    link = _model->GetLink(link_name_);
  }
  else
  {
    link = _model->GetLink();
    link_name_ = link->GetName();
  }

  if (!link)
  {
    ROS_FATAL("GazeboInclinometer plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  // default parameters
  frame_id_ = link_name_;

  if (_sdf->HasElement("frameId"))
    frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

  inclinometer_.header.frame_id = frame_id_;

  sensor_model_.Load(_sdf);

  if (_sdf->HasElement("GaussianNoise")) {
    double gaussianNoise;
    if (_sdf->GetElement("GaussianNoise")->GetValue()->Get(gaussianNoise) && gaussianNoise != 0.0) {
      sensor_model_.gaussian_noise = gaussianNoise;
    }
  }

  if (_sdf->HasElement("Drift")) {
    double drift;
    if (_sdf->GetElement("Drift")->GetValue()->Get(drift) && drift != 0.0) {
      sensor_model_.drift = drift;
    }
  }

#if (GAZEBO_MAJOR_VERSION >= 8)
  inclinometer_.orientation_covariance[0] = sensor_model_.drift.X()*sensor_model_.drift.X() + sensor_model_.gaussian_noise.X()*sensor_model_.gaussian_noise.X();
  inclinometer_.orientation_covariance[4] = sensor_model_.drift.Y()*sensor_model_.drift.Y() + sensor_model_.gaussian_noise.Y()*sensor_model_.gaussian_noise.Y();
  inclinometer_.orientation_covariance[8] = sensor_model_.drift.Z()*sensor_model_.drift.Z() + sensor_model_.gaussian_noise.Z()*sensor_model_.gaussian_noise.Z();
#else
  inclinometer_.orientation_covariance[0] = sensor_model_.drift.x*sensor_model_.drift.x + sensor_model_.gaussian_noise.x*sensor_model_.gaussian_noise.x;
  inclinometer_.orientation_covariance[4] = sensor_model_.drift.y*sensor_model_.drift.y + sensor_model_.gaussian_noise.y*sensor_model_.gaussian_noise.y;
  inclinometer_.orientation_covariance[8] = sensor_model_.drift.z*sensor_model_.drift.z + sensor_model_.gaussian_noise.z*sensor_model_.gaussian_noise.z;
#endif

  // start ros node
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  publisher_ = node_handle_->advertise<sensor_msgs::Imu>(topic_, 1);

  Reset();

  // connect Update function
  updateTimer.Load(world, _sdf);
  updateConnection = updateTimer.Connect(boost::bind(&GazeboInclinometer::Update, this));
}

void GazeboInclinometer::Reset()
{
  updateTimer.Reset();
  sensor_model_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboInclinometer::Update()
{
#if (GAZEBO_MAJOR_VERSION >= 8)
  common::Time sim_time = world->SimTime();
  double dt = updateTimer.getTimeSinceLastUpdate().Double();

  ignition::math::Pose3d pose = link->WorldPose();
  ignition::math::Vector3d orientation = sensor_model_(pose.Rot().Euler(), dt);

  inclinometer_.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
  inclinometer_.orientation = tf::createQuaternionMsgFromRollPitchYaw(orientation.X(),orientation.Y(),0);
#else
  common::Time sim_time = world->GetSimTime();
  double dt = updateTimer.getTimeSinceLastUpdate().Double();

  math::Pose pose = link->GetWorldPose();
  math::Vector3 orientation = sensor_model_(pose.rot.GetAsEuler()), dt);

  inclinometer_.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
  inclinometer_.orientation = tf::createQuaternionMsgFromRollPitchYaw(orientation.x,orientation.y,0);
#endif

  publisher_.publish(inclinometer_);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboInclinometer)

} // namespace gazebo
