#ifndef GAZEBO_PHYSICS_PLUGIN_HH
#define GAZEBO_PHYSICS_PLUGIN_HH

#include <string>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/advertise_options.h>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#define protected public
#include <gazebo/physics/physics.hh>
#undef protected
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/TransportIface.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include "gazebo_ext_msgs/GetCollisionNames.h"
#include "gazebo_ext_msgs/GetVisualNames.h"
#include "gazebo_ext_msgs/GetSurfaceParams.h"
#include "gazebo_ext_msgs/SetSurfaceParams.h"
#include "gazebo_ext_msgs/SetLinkColor.h"

namespace gazebo
{

class GazeboPhysicsPlugin : public WorldPlugin
{
  /// \brief Constructor
  public: GazeboPhysicsPlugin();

  /// \brief Destructor
  public: virtual ~GazeboPhysicsPlugin();

  // Documentation inherited
  protected: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

  // Documentation inherited
  protected: virtual void UpdateChild();

  /// \brief A pointer to the gazebo model.
  private: physics::WorldPtr world_;

  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;

  /// \brief A mutex to lock access to fields that are used in ROS message callbacks
  private: boost::mutex lock_;

  /// \brief A pointer to the Gazebo node
  private: transport::NodePtr gzNode;

  /// \brief A pointer to gazebo's ~/visual topic publisher
  private: transport::PublisherPtr visualPub;

  private: ros::ServiceServer get_col_name_srv_;
  private: ros::ServiceServer get_vis_name_srv_;
  private: ros::ServiceServer get_srv_;
  private: ros::ServiceServer set_srv_;
  private: ros::ServiceServer set_col_srv_;
  private: bool GetCollisionNamesCallback(gazebo_ext_msgs::GetCollisionNames::Request &req,
                                          gazebo_ext_msgs::GetCollisionNames::Response &res);
  private: bool GetVisualNamesCallback(gazebo_ext_msgs::GetVisualNames::Request &req,
                                       gazebo_ext_msgs::GetVisualNames::Response &res);
  private: bool GetSurfaceParamsCallback(gazebo_ext_msgs::GetSurfaceParams::Request &req,
                                         gazebo_ext_msgs::GetSurfaceParams::Response &res);
  private: bool SetSurfaceParamsCallback(gazebo_ext_msgs::SetSurfaceParams::Request &req,
                                         gazebo_ext_msgs::SetSurfaceParams::Response &res);
  private: bool SetLinkColorCallback(gazebo_ext_msgs::SetLinkColor::Request &req,
                                     gazebo_ext_msgs::SetLinkColor::Response &res);

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  private: void QueueThread();
  private: boost::thread callback_queue_thread_;
};
}
#endif