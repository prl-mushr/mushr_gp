cmake_minimum_required(VERSION 3.0.2)

project(mushr_gp)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  vesc_msgs
  geometry_msgs
  visualization_msgs
  message_generation
)

catkin_package()


