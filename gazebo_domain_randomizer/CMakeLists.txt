cmake_minimum_required(VERSION 2.8.3)
project(gazebo_domain_randomizer)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  gazebo_msgs
  std_msgs
)
