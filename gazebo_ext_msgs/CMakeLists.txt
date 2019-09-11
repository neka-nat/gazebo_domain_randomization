cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ext_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  std_srvs
  message_generation
  )

# add_message_files(
#   )

add_service_files(DIRECTORY srv FILES
  GetCollisionNames.srv
  GetVisualNames.srv
  SetSkyProperties.srv
  GetSkyProperties.srv
  SetLinkVisualProperties.srv
  GetLinkVisualProperties.srv
  SetLinkColor.srv
  SetSurfaceParams.srv
  GetSurfaceParams.srv
  )

generate_messages(DEPENDENCIES
  std_msgs
  )

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  std_srvs
)
