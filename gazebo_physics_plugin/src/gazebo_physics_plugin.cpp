#include <gazebo_physics_plugin/gazebo_physics_plugin.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GazeboPhysicsPlugin)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboPhysicsPlugin::GazeboPhysicsPlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboPhysicsPlugin::~GazeboPhysicsPlugin()
{
  // Finalize the controller
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboPhysicsPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  ROS_DEBUG_STREAM_NAMED("physics_plugin","Load");

  // save pointers
  this->world_ = _parent;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("physics_plugin", "A ROS node for Gazebo has not been initialized, unable to load plugin.");
    return;
  }

  this->rosnode_ = new ros::NodeHandle("~");

  // advertise services on the custom queue
  ros::AdvertiseServiceOptions get_aso =
    ros::AdvertiseServiceOptions::create<gazebo_ext_msgs::GetSurfaceParams>(
    "get_surface_params", boost::bind(&GazeboPhysicsPlugin::GetSurfaceParamsCallback,
    this, _1, _2), ros::VoidPtr(), &this->queue_);
  this->get_srv_ = this->rosnode_->advertiseService(get_aso);

  ros::AdvertiseServiceOptions set_aso =
    ros::AdvertiseServiceOptions::create<gazebo_ext_msgs::SetSurfaceParams>(
    "set_surface_params", boost::bind(&GazeboPhysicsPlugin::SetSurfaceParamsCallback,
    this, _1, _2), ros::VoidPtr(), &this->queue_);
  this->set_srv_ = this->rosnode_->advertiseService(set_aso);

  // start custom queue
  this->callback_queue_thread_ =
    boost::thread(boost::bind(&GazeboPhysicsPlugin::QueueThread, this));

  ROS_INFO_NAMED("physics_plugin", "Finished loading Gazebo Physics Plugin.");
}

////////////////////////////////////////////////////////////////////////////////
bool GazeboPhysicsPlugin::GetSurfaceParamsCallback(gazebo_ext_msgs::GetSurfaceParams::Request &req,
                                                   gazebo_ext_msgs::GetSurfaceParams::Response &res)
{
  boost::lock_guard<boost::mutex> lock(this->lock_);
#if GAZEBO_MAJOR_VERSION >= 8
  physics::CollisionPtr col = boost::dynamic_pointer_cast<physics::Collision>(world_->EntityByName(req.link_col_name));
#else
  physics::CollisionPtr col = boost::dynamic_pointer_cast<physics::Collision>(world_->GetEntity(req.link_col_name));
#endif
  if (!col)
  {
    res.success = false;
    res.status_message = "GetSurfaceParamsCallback: Could not access the collision!";
    return true;
  }
  physics::SurfaceParamsPtr surface = col->GetSurface();
  if (!surface)
  {
    res.success = false;
    res.status_message = "GetSurfaceParamsCallback: Could not access the surface!";
    return true;
  }
  physics::FrictionPyramidPtr friction = surface->FrictionPyramid();
  if (!friction)
  {
    res.success = false;
    res.status_message = "GetSurfaceParamsCallback: Could not access the friction!";
    return true;
  }
  res.elastic_modulus = friction->ElasticModulus();
  res.mu1 = friction->MuPrimary();
  res.mu2 = friction->MuSecondary();
  res.mu_torsion = friction->MuTorsion();
  res.patch_radius = friction->PatchRadius();
  res.poisson_ratio = friction->PoissonsRatio();
  res.success = true;
  return true;
}

bool GazeboPhysicsPlugin::SetSurfaceParamsCallback(gazebo_ext_msgs::SetSurfaceParams::Request &req,
                                                   gazebo_ext_msgs::SetSurfaceParams::Response &res)
{
  boost::lock_guard<boost::mutex> lock(this->lock_);
#if GAZEBO_MAJOR_VERSION >= 8
  physics::CollisionPtr col = boost::dynamic_pointer_cast<physics::Collision>(world_->EntityByName(req.link_col_name));
#else
  physics::CollisionPtr col = boost::dynamic_pointer_cast<physics::Collision>(world_->GetEntity(req.link_col_name));
#endif
  if (!col)
  {
    res.success = false;
    res.status_message = "GetSurfaceParamsCallback: Could not access the collision!";
    return true;
  }
  physics::SurfaceParamsPtr surface = col->GetSurface();
  if (!surface)
  {
    res.success = false;
    res.status_message = "GetSurfaceParamsCallback: Could not access the surface!";
    return true;
  }
  physics::FrictionPyramidPtr friction = surface->FrictionPyramid();
  if (!friction)
  {
    res.success = false;
    res.status_message = "GetSurfaceParamsCallback: Could not access the friction!";
    return true;
  }
  friction->SetElasticModulus(req.elastic_modulus);
  friction->SetMuPrimary(req.mu1);
  friction->SetMuSecondary(req.mu2);
  friction->SetMuTorsion(req.mu_torsion);
  friction->SetPatchRadius(req.patch_radius);
  friction->SetPoissonsRatio(req.poisson_ratio);
  res.success = true;
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboPhysicsPlugin::UpdateChild()
{
}

////////////////////////////////////////////////////////////////////////////////
void GazeboPhysicsPlugin::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}
}