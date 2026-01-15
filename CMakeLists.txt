cmake_minimum_required(VERSION 3.0.2)
project(table_plane_extractor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  genmsg
  sensor_msgs
  vision_msgs
  tf2_ros
  tf2_sensor_msgs
  visualization_msgs
  v4r_util
  robokudo_msgs
  actionlib
  grasping_pipeline_msgs
  object_detector_msgs
)

catkin_package()
include_directories()

