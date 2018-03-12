#include <gazebo/common/Events.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo_ros/gazebo_ros_api_plugin.h>

namespace gazebo
{

GazeboRosExtensionPlugin::GazeboRosExtensionPlugin() :
  world_created_(false),
  stop_(false),
  plugin_loaded_(false)
{
  robot_namespace_.clear();
}

GazeboRosExtentionPlugin::~GazeboRosExtensionPlugin()
{
  ROS_DEBUG_STREAM_NAMED("api_plugin","GazeboRosExtensionPlugin Deconstructor start");

  // Unload the sigint event
  sigint_event_.reset();
  ROS_DEBUG_STREAM_NAMED("api_plugin","After sigint_event unload");

  // Don't attempt to unload this plugin if it was never loaded in the Load() function
  if(!plugin_loaded_)
  {
    ROS_DEBUG_STREAM_NAMED("api_plugin","Deconstructor skipped because never loaded");
    return;
  }

  // Disconnect slots
  load_gazebo_ros_api_plugin_event_.reset();
  ROS_DEBUG_STREAM_NAMED("api_plugin","Slots disconnected");

  // Stop the multi threaded ROS spinner
  async_ros_spin_->stop();
  ROS_DEBUG_STREAM_NAMED("api_plugin","Async ROS Spin Stopped");

  // Shutdown the ROS node
  nh_->shutdown();
  ROS_DEBUG_STREAM_NAMED("api_plugin","Node Handle Shutdown");

  // Shutdown ROS queue
  gazebo_callback_queue_thread_->join();
  ROS_DEBUG_STREAM_NAMED("api_plugin","Callback Queue Joined");

  ROS_DEBUG_STREAM_NAMED("api_plugin","Unloaded");
}

void GazeboRosApiPlugin::shutdownSignal()
{
  ROS_DEBUG_STREAM_NAMED("api_plugin","shutdownSignal() recieved");
  stop_ = true;
}

void GazeboRosApiPlugin::Load(int argc, char** argv)
{
  ROS_DEBUG_STREAM_NAMED("api_plugin","Load");

  // connect to sigint event
  sigint_event_ = gazebo::event::Events::ConnectSigInt(boost::bind(&GazeboRosApiPlugin::shutdownSignal,this));

  // setup ros related
  if (!ros::isInitialized())
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler);
  else
    ROS_ERROR_NAMED("api_plugin", "Something other than this gazebo_ros_api plugin started ros::init(...), command line arguments may not be parsed properly.");

  // check if the ros master is available - required
  while(!ros::master::check())
  {
    ROS_WARN_STREAM_NAMED("api_plugin","No ROS master - start roscore to continue...");
    // wait 0.5 second
    usleep(500*1000); // can't use ROS Time here b/c node handle is not yet initialized

    if(stop_)
    {
      ROS_WARN_STREAM_NAMED("api_plugin","Canceled loading Gazebo ROS API plugin by sigint event");
      return;
    }
  }

  nh_.reset(new ros::NodeHandle("~")); // advertise topics and services in this node's namespace

  // Built-in multi-threaded ROS spinning
  async_ros_spin_.reset(new ros::AsyncSpinner(0)); // will use a thread for each CPU core
  async_ros_spin_->start();

  /// \brief setup custom callback queue
  gazebo_callback_queue_thread_.reset(new boost::thread( &GazeboRosApiPlugin::gazeboQueueThread, this) );

  /// \brief start a thread for the physics dynamic reconfigure node
  physics_reconfigure_thread_.reset(new boost::thread(boost::bind(&GazeboRosApiPlugin::physicsReconfigureThread, this)));

  // below needs the world to be created first
  load_gazebo_ros_api_plugin_event_ = gazebo::event::Events::ConnectWorldCreated(boost::bind(&GazeboRosApiPlugin::loadGazeboRosApiPlugin,this,_1));

  plugin_loaded_ = true;
  ROS_INFO_NAMED("api_plugin", "Finished loading Gazebo ROS API Plugin.");
}

void GazeboRosApiPlugin::loadGazeboRosApiPlugin(std::string world_name)
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
    ROS_FATAL_NAMED("api_plugin", "cannot load gazebo ros api server plugin, physics::get_world() fails to return world");
    return;
  }

  gazebonode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gazebonode_->Init(world_name);
  //stat_sub_ = gazebonode_->Subscribe("~/world_stats", &GazeboRosApiPlugin::publishSimTime, this); // TODO: does not work in server plugin?
  factory_pub_ = gazebonode_->Advertise<gazebo::msgs::Factory>("~/factory");
  factory_light_pub_ = gazebonode_->Advertise<gazebo::msgs::Light>("~/factory/light");
  light_modify_pub_ = gazebonode_->Advertise<gazebo::msgs::Light>("~/light/modify");
  request_pub_ = gazebonode_->Advertise<gazebo::msgs::Request>("~/request");
  response_sub_ = gazebonode_->Subscribe("~/response",&GazeboRosApiPlugin::onResponse, this);

  // reset topic connection counts
  pub_link_states_connection_count_ = 0;
  pub_model_states_connection_count_ = 0;

  /// \brief advertise all services
  advertiseServices();

  // hooks for applying forces, publishing simtime on /clock
  wrench_update_event_ = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosApiPlugin::wrenchBodySchedulerSlot,this));
  force_update_event_  = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosApiPlugin::forceJointSchedulerSlot,this));
  time_update_event_   = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosApiPlugin::publishSimTime,this));
}

void GazeboRosApiPlugin::gazeboQueueThread()
{
  static const double timeout = 0.001;
  while (nh_->ok())
  {
    gazebo_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void GazeboRosApiPlugin::advertiseServices()
{
  // publish clock for simulated ros time
  pub_clock_ = nh_->advertise<rosgraph_msgs::Clock>("/clock",10);

  // Advertise more services on the custom queue
  std::string get_model_properties_service_name("get_sky_properties");
  ros::AdvertiseServiceOptions get_sky_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetModelProperties>(
                                                                          get_model_properties_service_name,
                                                                          boost::bind(&GazeboRosApiPlugin::getModelProperties,this,_1,_2),
                                                                          ros::VoidPtr(), &gazebo_queue_);
  get_model_properties_service_ = nh_->advertiseService(get_sky_properties_aso);

  // Advertise more services on the custom queue
  std::string set_light_properties_service_name("set_light_properties");
  ros::AdvertiseServiceOptions set_light_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetLightProperties>(
                                                                          set_light_properties_service_name,
                                                                          boost::bind(&GazeboRosApiPlugin::setLightProperties,this,_1,_2),
                                                                          ros::VoidPtr(), &gazebo_queue_);
  set_light_properties_service_ = nh_->advertiseService(set_light_properties_aso);

  last_pub_clock_time_ = world_->GetSimTime();
}

bool GazeboRosApiPlugin::getSkyProperties(gazebo_msgs::GetLightProperties::Request &req,
                                               gazebo_msgs::GetLightProperties::Response &res)
{
  gazebo::physics::LightPtr phy_light = world_->Light(req.light_name);

  if (phy_light == NULL)
  {
      res.success = false;
      res.status_message = "getLightProperties: Requested light " + req.light_name + " not found!";
  }
  else
  {
    gazebo::msgs::Light light;
    phy_light->FillMsg(light);

    res.diffuse.r = light.diffuse().r();
    res.diffuse.g = light.diffuse().g();
    res.diffuse.b = light.diffuse().b();
    res.diffuse.a = light.diffuse().a();

    res.attenuation_constant = light.attenuation_constant();
    res.attenuation_linear = light.attenuation_linear();
    res.attenuation_quadratic = light.attenuation_quadratic();

    res.success = true;
  }

  return true;
}

bool GazeboRosApiPlugin::setSkyProperties(gazebo_msgs::SetLightProperties::Request &req,
                                               gazebo_msgs::SetLightProperties::Response &res)
{
  gazebo::physics::LightPtr phy_light = world_->Light(req.light_name);

  if (phy_light == NULL)
  {
    res.success = false;
    res.status_message = "setLightProperties: Requested light " + req.light_name + " not found!";
  }
  else
  {
    gazebo::msgs::Light light;

    phy_light->FillMsg(light);

    light.mutable_diffuse()->set_r(req.diffuse.r);
    light.mutable_diffuse()->set_g(req.diffuse.g);
    light.mutable_diffuse()->set_b(req.diffuse.b);
    light.mutable_diffuse()->set_a(req.diffuse.a);

    light.set_attenuation_constant(req.attenuation_constant);
    light.set_attenuation_linear(req.attenuation_linear);
    light.set_attenuation_quadratic(req.attenuation_quadratic);

    light_modify_pub_->Publish(light, true);

    res.success = true;
  }

  return true;
}

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosExtensionPlugin)
}
