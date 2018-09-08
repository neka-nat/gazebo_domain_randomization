#include <gazebo/common/Events.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/ogre_gazebo.h>
#include "skyx/include/SkyX.h"
#include "gazebo_scene_plugin/gazebo_scene_plugin.h"

namespace gazebo
{

GazeboScenePlugin::GazeboScenePlugin() :
  world_created_(false),
  stop_(false),
  plugin_loaded_(false)
{
  robot_namespace_.clear();
}

GazeboScenePlugin::~GazeboScenePlugin()
{
  ROS_DEBUG_STREAM_NAMED("scene_plugin","GazeboScenePlugin Deconstructor start");

  // Unload the sigint event
  sigint_event_.reset();
  ROS_DEBUG_STREAM_NAMED("scene_plugin","After sigint_event unload");

  // Don't attempt to unload this plugin if it was never loaded in the Load() function
  if(!plugin_loaded_)
  {
    ROS_DEBUG_STREAM_NAMED("scene_plugin", "Deconstructor skipped because never loaded");
    return;
  }

  // Shutdown the ROS node
  nh_->shutdown();
  ROS_DEBUG_STREAM_NAMED("scene_plugin", "Node Handle Shutdown");

  // Shutdown ROS queue
  gazebo_callback_init_thread_->join();
  ROS_DEBUG_STREAM_NAMED("scene_plugin", "Callback Init Joined");

  ROS_DEBUG_STREAM_NAMED("scene_plugin", "Unloaded");
}

void GazeboScenePlugin::shutdownSignal()
{
  ROS_DEBUG_STREAM_NAMED("scene_plugin","shutdownSignal() recieved");
  stop_ = true;
}

void GazeboScenePlugin::Load(int argc, char** argv)
{
  ROS_DEBUG_STREAM_NAMED("scene_plugin","Load");

  // connect to sigint event
  sigint_event_ = gazebo::event::Events::ConnectSigInt(boost::bind(&GazeboScenePlugin::shutdownSignal,this));

  // setup ros related
  if (!ros::isInitialized())
    ros::init(argc,argv, "gazebo", ros::init_options::NoSigintHandler);
  else
    ROS_WARN_NAMED("scene_plugin", "Something other than this gazebo_scene plugin started ros::init(...), command line arguments may not be parsed properly.");

  // check if the ros master is available - required
  while(!ros::master::check())
  {
    ROS_WARN_STREAM_NAMED("scene_plugin", "No ROS master - start roscore to continue...");
    // wait 0.5 second
    usleep(500*1000); // can't use ROS Time here b/c node handle is not yet initialized

    if(stop_)
    {
      ROS_WARN_STREAM_NAMED("scene_plugin", "Canceled loading Gazebo ROS API plugin by sigint event");
      return;
    }
  }

  nh_.reset(new ros::NodeHandle("~")); // advertise topics and services in this node's namespace

  /// \brief setup custom callback
  gazebo_callback_init_thread_.reset(new boost::thread( &GazeboScenePlugin::gazeboInitThread, this) );
  update_connection = event::Events::ConnectPreRender(boost::bind(&GazeboScenePlugin::Update, this));

  plugin_loaded_ = true;
  ROS_INFO_NAMED("scene_plugin", "Finished loading Gazebo Scene Plugin.");
}

void GazeboScenePlugin::gazeboInitThread()
{
  rendering::ScenePtr scene = rendering::get_scene();
  while (!scene || !scene->Initialized())
  {
    usleep(500*1000); // can't use ROS Time here b/c node handle is not yet initialized
    scene = rendering::get_scene();
  }
  advertiseServices();
}

void GazeboScenePlugin::Update()
{
  static const double timeout = 0.001;
  if (nh_->ok())
  {
    gazebo_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void GazeboScenePlugin::advertiseServices()
{
  // Advertise more services on the custom queue
  std::string get_sky_properties_service_name("get_sky_properties");
  ros::AdvertiseServiceOptions get_sky_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_ext_msgs::GetSkyProperties>(
                                                                            get_sky_properties_service_name,
                                                                            boost::bind(&GazeboScenePlugin::getSkyProperties,this,_1,_2),
                                                                            ros::VoidPtr(), &gazebo_queue_);
  get_sky_properties_service_ = nh_->advertiseService(get_sky_properties_aso);

  std::string set_sky_properties_service_name("set_sky_properties");
  ros::AdvertiseServiceOptions set_sky_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_ext_msgs::SetSkyProperties>(
                                                                            set_sky_properties_service_name,
                                                                            boost::bind(&GazeboScenePlugin::setSkyProperties,this,_1,_2),
                                                                            ros::VoidPtr(), &gazebo_queue_);
  set_sky_properties_service_ = nh_->advertiseService(set_sky_properties_aso);

  std::string get_link_visual_properties_service_name("get_link_visual_properties");
  ros::AdvertiseServiceOptions get_link_visual_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_ext_msgs::GetLinkVisualProperties>(
                                                                                   get_link_visual_properties_service_name,
                                                                                   boost::bind(&GazeboScenePlugin::getLinkVisualProperties,this,_1,_2),
                                                                                   ros::VoidPtr(), &gazebo_queue_);
  get_link_visual_properties_service_ = nh_->advertiseService(get_link_visual_properties_aso);

  std::string set_link_visual_properties_service_name("set_link_visual_properties");
  ros::AdvertiseServiceOptions set_link_visual_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_ext_msgs::SetLinkVisualProperties>(
                                                                                   set_link_visual_properties_service_name,
                                                                                   boost::bind(&GazeboScenePlugin::setLinkVisualProperties,this,_1,_2),
                                                                                   ros::VoidPtr(), &gazebo_queue_);
  set_link_visual_properties_service_ = nh_->advertiseService(set_link_visual_properties_aso);
}

bool GazeboScenePlugin::getSkyProperties(gazebo_ext_msgs::GetSkyProperties::Request &req,
                                         gazebo_ext_msgs::GetSkyProperties::Response &res)
{
  boost::lock_guard<boost::mutex> lock(this->lock_);

  // Get scene pointer
  rendering::ScenePtr scene = rendering::get_scene();

  // Wait until the scene is initialized.
  if (!scene || !scene->Initialized() || !scene->GetSkyX())
  {
    res.success = false;
    res.status_message = "getSkyProperties: Could not access the scene!";
  }
  else
  {
    SkyX::VClouds::VClouds *vclouds =
      scene->GetSkyX()->getVCloudsManager()->getVClouds();
    SkyX::BasicController *controller =
      dynamic_cast<SkyX::BasicController *>(scene->GetSkyX()->getController());

    Ogre::Vector3 t = controller->getTime();
    res.time = t.x;
    res.sunrise = t.y;
    res.sunset = t.z;
 
    res.wind_speed = vclouds->getWindSpeed();
    res.wind_direction = vclouds->getWindDirection().valueRadians();
    Ogre::Vector4 vec4 = vclouds->getAmbientFactors();
    res.cloud_ambient.r = vec4.x;
    res.cloud_ambient.g = vec4.y;
    res.cloud_ambient.b = vec4.z;
    res.cloud_ambient.a = vec4.w;

    Ogre::Vector2 wheater = vclouds->getWheater();
    res.humidity = wheater.x;
    res.mean_cloud_size = wheater.y;
    res.success = true;
  }

  return true;
}

bool GazeboScenePlugin::setSkyProperties(gazebo_ext_msgs::SetSkyProperties::Request &req,
                                         gazebo_ext_msgs::SetSkyProperties::Response &res)
{
  boost::lock_guard<boost::mutex> lock(this->lock_);

  // Get scene pointer
  rendering::ScenePtr scene = rendering::get_scene();

  // Wait until the scene is initialized.
  if (!scene || !scene->Initialized() || !scene->GetSkyX())
  {
    res.success = false;
    res.status_message = "setSkyProperties: Could not access the scene!";
  }
  else
  {
    Ogre::Root::getSingletonPtr()->addFrameListener(scene->GetSkyX());
    scene->GetSkyX()->update(0);

    scene->GetSkyX()->setVisible(true);

    SkyX::VClouds::VClouds *vclouds =
      scene->GetSkyX()->getVCloudsManager()->getVClouds();
    SkyX::BasicController *controller =
      dynamic_cast<SkyX::BasicController *>(scene->GetSkyX()->getController());

    Ogre::Vector3 t;
    t.x = ignition::math::clamp(req.time, 0.0, 24.0);
    t.y = ignition::math::clamp(req.sunrise, 0.0, 24.0);
    t.z = ignition::math::clamp(req.sunset, 0.0, 24.0);
    controller->setTime(t);

    vclouds->setWindSpeed(req.wind_speed);
    vclouds->setWindDirection(Ogre::Radian(req.wind_direction));
    vclouds->setAmbientFactors(Ogre::Vector4(
          req.cloud_ambient.r,
          req.cloud_ambient.g,
          req.cloud_ambient.b,
          req.cloud_ambient.a));

    vclouds->setWheater(ignition::math::clamp(req.humidity, 0.0, 1.0),
      ignition::math::clamp(req.mean_cloud_size, 0.0, 1.0), true);

    scene->GetSkyX()->update(0);
    res.success = true;
  }

  return true;
}

bool GazeboScenePlugin::getLinkVisualProperties(gazebo_ext_msgs::GetLinkVisualProperties::Request &req,
                                                gazebo_ext_msgs::GetLinkVisualProperties::Response &res)
{
  boost::lock_guard<boost::mutex> lock(this->lock_);

  // Get scene pointer
  rendering::ScenePtr scene = rendering::get_scene();

  // Wait until the scene is initialized.
  if (!scene || !scene->Initialized())
  {
    res.success = false;
    res.status_message = "getLinkVisualProperties: Could not access the scene!";
    return true;
  }
  rendering::VisualPtr visual = scene->GetVisual(req.link_visual_name);
  if (!visual)
  {
    res.success = false;
    res.status_message = "getLinkVisualProperties: Could not access the visual!";
    return true;
  }
#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Color ambient = visual->Ambient();
  ignition::math::Color diffuse = visual->Diffuse();
  ignition::math::Color specular = visual->Specular();
  ignition::math::Color emissive = visual->Emissive();
  res.ambient.r = ambient.R();
  res.ambient.g = ambient.G();
  res.ambient.b = ambient.B();
  res.ambient.a = ambient.A();
  res.diffuse.r = diffuse.R();
  res.diffuse.g = diffuse.G();
  res.diffuse.b = diffuse.B();
  res.diffuse.a = diffuse.A();
  res.specular.r = specular.R();
  res.specular.g = specular.G();
  res.specular.b = specular.B();
  res.specular.a = specular.A();
  res.emissive.r = emissive.R();
  res.emissive.g = emissive.G();
  res.emissive.b = emissive.B();
  res.emissive.a = emissive.A();
#else
  common::Color ambient = visual->GetAmbient();
  common::Color diffuse = visual->GetDiffuse();
  common::Color specular = visual->GetSpecular();
  common::Color emissive = visual->GetEmissive();
  res.ambient.r = ambient.r;
  res.ambient.g = ambient.g;
  res.ambient.b = ambient.b;
  res.ambient.a = ambient.a;
  res.diffuse.r = diffuse.r;
  res.diffuse.g = diffuse.g;
  res.diffuse.b = diffuse.b;
  res.diffuse.a = diffuse.a;
  res.specular.r = specular.r;
  res.specular.g = specular.g;
  res.specular.b = specular.b;
  res.specular.a = specular.a;
  res.emissive.r = emissive.r;
  res.emissive.g = emissive.g;
  res.emissive.b = emissive.b;
  res.emissive.a = emissive.a;
#endif
  res.success = true;
  return true;
}

bool GazeboScenePlugin::setLinkVisualProperties(gazebo_ext_msgs::SetLinkVisualProperties::Request &req,
                                                gazebo_ext_msgs::SetLinkVisualProperties::Response &res)
{
  boost::lock_guard<boost::mutex> lock(this->lock_);

  // Get scene pointer
  rendering::ScenePtr scene = rendering::get_scene();

  // Wait until the scene is initialized.
  if (!scene || !scene->Initialized())
  {
    res.success = false;
    res.status_message = "setLinkVisualProperties: Could not access the scene!";
    return true;
  }
  rendering::VisualPtr visual = scene->GetVisual(req.link_visual_name);
  if (!visual)
  {
    res.success = false;
    res.status_message = "setLinkVisualProperties: Could not access the visual!";
    return true;
  }
#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Color ambient(req.ambient.r, req.ambient.g, req.ambient.b, req.ambient.a);
  ignition::math::Color diffuse(req.diffuse.r, req.diffuse.g, req.diffuse.b, req.diffuse.a);
  ignition::math::Color specular(req.specular.r, req.specular.g, req.specular.b, req.specular.a);
  ignition::math::Color emissive(req.emissive.r, req.emissive.g, req.emissive.b, req.emissive.a);
#else
  common::Color ambient(req.ambient.r, req.ambient.g, req.ambient.b, req.ambient.a);
  common::Color diffuse(req.diffuse.r, req.diffuse.g, req.diffuse.b, req.diffuse.a);
  common::Color specular(req.specular.r, req.specular.g, req.specular.b, req.specular.a);
  common::Color emissive(req.emissive.r, req.emissive.g, req.emissive.b, req.emissive.a);
#endif
  visual->SetAmbient(ambient);
  visual->SetDiffuse(diffuse);
  visual->SetSpecular(specular);
  visual->SetEmissive(emissive);
  res.success = true;
  return true;
}

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboScenePlugin)
}
