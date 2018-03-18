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
    ROS_DEBUG_STREAM_NAMED("scene_plugin","Deconstructor skipped because never loaded");
    return;
  }

  // Disconnect slots
  load_gazebo_scene_plugin_event_.reset();
  ROS_DEBUG_STREAM_NAMED("scene_plugin","Slots disconnected");

  // Stop the multi threaded ROS spinner
  async_ros_spin_->stop();
  ROS_DEBUG_STREAM_NAMED("scene_plugin","Async ROS Spin Stopped");

  // Shutdown the ROS node
  nh_->shutdown();
  ROS_DEBUG_STREAM_NAMED("scene_plugin","Node Handle Shutdown");

  // Shutdown ROS queue
  gazebo_callback_queue_thread_->join();
  ROS_DEBUG_STREAM_NAMED("scene_plugin","Callback Queue Joined");

  ROS_DEBUG_STREAM_NAMED("scene_plugin","Unloaded");
}

void GazeboScenePlugin::shutdownSignal()
{
  ROS_DEBUG_STREAM_NAMED("scene_plugin","shutdownSignal() recieved");
  stop_ = true;
}

void GazeboScenePlugin::Load(int argc, char** argv)
{
  ROS_DEBUG_STREAM_NAMED("extension_plugin","Load");

  // connect to sigint event
  sigint_event_ = gazebo::event::Events::ConnectSigInt(boost::bind(&GazeboScenePlugin::shutdownSignal,this));

  // setup ros related
  if (!ros::isInitialized())
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler);
  else
    ROS_ERROR_NAMED("extension_plugin", "Something other than this gazebo_ros_api plugin started ros::init(...), command line arguments may not be parsed properly.");

  // check if the ros master is available - required
  while(!ros::master::check())
  {
    ROS_WARN_STREAM_NAMED("extension_plugin","No ROS master - start roscore to continue...");
    // wait 0.5 second
    usleep(500*1000); // can't use ROS Time here b/c node handle is not yet initialized

    if(stop_)
    {
      ROS_WARN_STREAM_NAMED("extension_plugin","Canceled loading Gazebo ROS API plugin by sigint event");
      return;
    }
  }

  nh_.reset(new ros::NodeHandle("~")); // advertise topics and services in this node's namespace

  // Built-in multi-threaded ROS spinning
  async_ros_spin_.reset(new ros::AsyncSpinner(0)); // will use a thread for each CPU core
  async_ros_spin_->start();

  /// \brief setup custom callback queue
  gazebo_callback_queue_thread_.reset(new boost::thread( &GazeboScenePlugin::gazeboQueueThread, this) );

  // below needs the world to be created first
  load_gazebo_scene_plugin_event_ = gazebo::event::Events::ConnectWorldCreated(boost::bind(&GazeboScenePlugin::loadGazeboScenePlugin,this,_1));

  plugin_loaded_ = true;
  ROS_INFO_NAMED("extension_plugin", "Finished loading Gazebo ROS API Plugin.");
}

void GazeboScenePlugin::loadGazeboScenePlugin(std::string world_name)
{
  // make sure things are only called once
  lock_.lock();
  if (world_created_)
  {
    lock_.unlock();
    return;
  }

  // set flag to true and load this plugin
  world_created_ = true;
  lock_.unlock();

  world_ = gazebo::physics::get_world(world_name);
  if (!world_)
  {
    //ROS_ERROR_NAMED("api_plugin", "world name: [%s]",world->GetName().c_str());
    // connect helper function to signal for scheduling torque/forces, etc
    ROS_FATAL_NAMED("extension_plugin", "cannot load gazebo ros api server plugin, physics::get_world() fails to return world");
    return;
  }

  gazebonode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gazebonode_->Init(world_name);

  /// \brief advertise all services
  advertiseServices();
}

void GazeboScenePlugin::gazeboQueueThread()
{
  static const double timeout = 0.001;
  while (nh_->ok())
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

  // Advertise more services on the custom queue
  std::string set_sky_properties_service_name("set_sky_properties");
  ros::AdvertiseServiceOptions set_sky_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_ext_msgs::SetSkyProperties>(
                                                                            set_sky_properties_service_name,
                                                                            boost::bind(&GazeboScenePlugin::setSkyProperties,this,_1,_2),
                                                                        ros::VoidPtr(), &gazebo_queue_);
  set_sky_properties_service_ = nh_->advertiseService(set_sky_properties_aso);
}

bool GazeboScenePlugin::getSkyProperties(gazebo_ext_msgs::GetSkyProperties::Request &req,
                                         gazebo_ext_msgs::GetSkyProperties::Response &res)
{
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

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboScenePlugin)
}
