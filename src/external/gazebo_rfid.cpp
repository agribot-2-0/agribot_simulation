#include <agribot_simulation/external/gazebo_rfid.h>
#include <gazebo/common/Events.hh>


namespace gazebo
{
  GazeboRfid::GazeboRfid()
  {
  }

  GazeboRfid::~GazeboRfid()
  {
  }

  void GazeboRfid::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
  }

  void GazeboRfid::Reset()
  {
  }

  void GazeboRfid::Update()
  {
  }
  GZ_REGISTER_MODEL_PLUGIN(GazeboRfid)
}