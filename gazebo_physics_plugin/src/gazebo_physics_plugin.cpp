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
  //Finalize gazebo node
  this->gzNode->Fini();

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

  //Start Gazebo node and create ~/visual publisher
  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();
  this->visualPub = this->gzNode->Advertise<msgs::Visual>("~/visual");

  // advertise services on the custom queue
  ros::AdvertiseServiceOptions get_col_name_aso =
    ros::AdvertiseServiceOptions::create<gazebo_ext_msgs::GetCollisionNames>(
    "get_collision_names", boost::bind(&GazeboPhysicsPlugin::GetCollisionNamesCallback,
    this, _1, _2), ros::VoidPtr(), &this->queue_);
  this->get_col_name_srv_ = this->rosnode_->advertiseService(get_col_name_aso);

  ros::AdvertiseServiceOptions get_vis_names_aso =
    ros::AdvertiseServiceOptions::create<gazebo_ext_msgs::GetVisualNames>(
    "get_visual_names", boost::bind(&GazeboPhysicsPlugin::GetVisualNamesCallback,
    this,_1,_2), ros::VoidPtr(), &queue_);
  this->get_vis_name_srv_ = this->rosnode_->advertiseService(get_vis_names_aso);

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

  ros::AdvertiseServiceOptions set_col_aso =
    ros::AdvertiseServiceOptions::create<gazebo_ext_msgs::SetLinkColor>(
    "set_link_color", boost::bind(&GazeboPhysicsPlugin::SetLinkColorCallback,
    this, _1, _2), ros::VoidPtr(), &this->queue_);
  this->set_col_srv_ = this->rosnode_->advertiseService(set_col_aso);

  // start custom queue
  this->callback_queue_thread_ =
    boost::thread(boost::bind(&GazeboPhysicsPlugin::QueueThread, this));

  ROS_INFO_NAMED("physics_plugin", "Finished loading Gazebo Physics Plugin.");
}

bool GazeboPhysicsPlugin::GetCollisionNamesCallback(gazebo_ext_msgs::GetCollisionNames::Request &req,
                                                    gazebo_ext_msgs::GetCollisionNames::Response &res)
{
  boost::lock_guard<boost::mutex> lock(this->lock_);
  std::vector<std::string> col_names;
  for (std::vector<std::string>::const_iterator itr = req.link_names.begin(); itr != req.link_names.end(); ++itr)
  {
#if GAZEBO_MAJOR_VERSION >= 8
    physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(world_->EntityByName(*itr));
#else
    physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(world_->GetEntity(*itr));
#endif
    if (!link)
    {
      res.success = false;
      res.status_message = "GetCollisionNamesCallback: Could not access the link!";
      return true;
    }
    physics::Collision_V cols = link->GetCollisions();
    for (physics::Collision_V::const_iterator jtr = cols.begin(); jtr != cols.end(); ++jtr)
    {
      col_names.push_back((*itr) + "::" + (*jtr)->GetName());
    }
  }
  res.link_collision_names = col_names;
  res.success = true;
  return true;
}

bool GazeboPhysicsPlugin::GetVisualNamesCallback(gazebo_ext_msgs::GetVisualNames::Request &req,
                                                 gazebo_ext_msgs::GetVisualNames::Response &res)
{
  boost::lock_guard<boost::mutex> lock(this->lock_);
  std::vector<std::string> vis_names;
  std::vector<std::string> parent_names;
  for (std::vector<std::string>::const_iterator itr = req.link_names.begin(); itr != req.link_names.end(); ++itr)
  {
#if GAZEBO_MAJOR_VERSION >= 8
    physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(world_->EntityByName(*itr));
#else
    physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(world_->GetEntity(*itr));
#endif
    if (!link)
    {
      res.success = false;
      res.status_message = "GetCollisionNamesCallback: Could not access the link!";
      return true;
    }
    for (physics::Link::Visuals_M::const_iterator jtr = link->visuals.begin(); jtr != link->visuals.end(); ++jtr)
    {
      vis_names.push_back(jtr->second.name());
      parent_names.push_back(*itr);
    }
  }
  res.link_visual_names = vis_names;
  res.link_parent_names = parent_names;
  res.success = true;
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool GazeboPhysicsPlugin::GetSurfaceParamsCallback(gazebo_ext_msgs::GetSurfaceParams::Request &req,
                                                   gazebo_ext_msgs::GetSurfaceParams::Response &res)
{
  boost::lock_guard<boost::mutex> lock(this->lock_);
#if GAZEBO_MAJOR_VERSION >= 8
  physics::CollisionPtr col = boost::dynamic_pointer_cast<physics::Collision>(world_->EntityByName(req.link_collision_name));
#else
  physics::CollisionPtr col = boost::dynamic_pointer_cast<physics::Collision>(world_->GetEntity(req.link_collision_name));
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
  physics::CollisionPtr col = boost::dynamic_pointer_cast<physics::Collision>(world_->EntityByName(req.link_collision_name));
#else
  physics::CollisionPtr col = boost::dynamic_pointer_cast<physics::Collision>(world_->GetEntity(req.link_collision_name));
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
bool GazeboPhysicsPlugin::SetLinkColorCallback(gazebo_ext_msgs::SetLinkColor::Request &req,
                                               gazebo_ext_msgs::SetLinkColor::Response &res)
{
  boost::lock_guard<boost::mutex> lock(this->lock_);

  //gets the visual message
#if GAZEBO_MAJOR_VERSION >= 8
  physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(world_->EntityByName(req.link_parent_name));
#else
  physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(world_->GetEntity(req.link_parent_name));
#endif

  msgs::Visual visualMsg = link->GetVisualMessage(req.link_visual_name);
  visualMsg.set_name(link->GetScopedName());

  //starts a new material, in case the object doesn't have one
  if ((!visualMsg.has_material()) || visualMsg.mutable_material() == NULL) {
    msgs::Material *materialMsg = new msgs::Material;
    visualMsg.set_allocated_material(materialMsg);
  }

  // Set color
  common::Color ambient(req.ambient.r, req.ambient.g, req.ambient.b, req.ambient.a);
  common::Color diffuse(req.diffuse.r, req.diffuse.g, req.diffuse.b, req.diffuse.a);

  msgs::Color *colorMsg = new msgs::Color(msgs::Convert(ambient));
  msgs::Color *diffuseMsg = new msgs::Color(msgs::Convert(diffuse));

  //add color to material
  msgs::Material *materialMsg = visualMsg.mutable_material();
  if (materialMsg->has_ambient())
  {
    materialMsg->clear_ambient();
  }
  materialMsg->set_allocated_ambient(colorMsg);
  if (materialMsg->has_diffuse())
  {
    materialMsg->clear_diffuse();
  }
  materialMsg->set_allocated_diffuse(diffuseMsg);

  //publish message
  this->visualPub->Publish(visualMsg);

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