/*
 * ur10e_collision_free_demo_node.cpp
 *
 *  Created on: Nov 23, 2020
 *      Author: yik
 */

#include <ur10e_collision_free_demo/ur10e_collision_free_demo.h>

using namespace online_planning_test;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "online_planning_test");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  bool plotting = true;
  bool rviz = true;

  // Get ROS Parameters
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);

  TestExample example(nh, plotting, rviz);
  example.parsing_data("/home/yik/catkin_ws/src/SDU-Collision-Free-Demo/ur10e_collision_free_demo/config/parameters.yaml");
  example.run();
}
