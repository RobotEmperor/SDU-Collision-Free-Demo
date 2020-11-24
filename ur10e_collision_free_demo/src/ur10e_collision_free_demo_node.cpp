/*
 * ur10e_collision_free_demo_node.cpp
 *
 *  Created on: Nov 23, 2020
 *      Author: yik
 */

#include <ur10e_collision_free_demo/ur10e_collision_free_demo.h>

using namespace ur10e_collision_free_demo;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur10e_collision_free_demo");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  bool plotting = true;
  bool rviz = true;
  int steps = 12;
  double box_size = 0.002;

  // Get ROS Parameters
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);
  pnh.param<int>("steps", steps, steps);
  pnh.param<double>("box_size", box_size, box_size);

  OnlinePlanningExample example(nh, plotting, rviz, steps, box_size);
  example.run();
}
