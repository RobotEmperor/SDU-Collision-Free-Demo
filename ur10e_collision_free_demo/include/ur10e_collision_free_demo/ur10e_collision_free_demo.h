/*
 * ur10e_collision_free_demo.h
 *
 *  Created on: Nov 23, 2020
 *      Author: yik
 */

#ifndef SDU_COLLISION_FREE_DEMO_UR10E_COLLISION_FREE_DEMO_INCLUDE_UR10E_COLLISION_FREE_DEMO_UR10E_COLLISION_FREE_DEMO_H_
#define SDU_COLLISION_FREE_DEMO_UR10E_COLLISION_FREE_DEMO_INCLUDE_UR10E_COLLISION_FREE_DEMO_UR10E_COLLISION_FREE_DEMO_H_

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <ur10e_collision_free_demo/example.h>
#include <rw/math.hpp>

#define DEGREE2RADIAN M_PI/180.0
#define RADIAN2DEGREE  180.0/M_PI

namespace online_planning_test
{
/**
 * @brief An example of a robot picking up a box and placing it on a shelf leveraging
 * tesseract and trajopt to generate the motion trajectory.
 */
class PickAndPlaceExample : public Example
{
public:
  PickAndPlaceExample(const ros::NodeHandle& nh, bool plotting, bool rviz);
  ~PickAndPlaceExample() override = default;
  PickAndPlaceExample(const PickAndPlaceExample&) = default;
  PickAndPlaceExample& operator=(const PickAndPlaceExample&) = default;
  PickAndPlaceExample(PickAndPlaceExample&&) = default;
  PickAndPlaceExample& operator=(PickAndPlaceExample&&) = default;

  void make_circle_waypoints(int direction_, double radious_);
  bool run() override;

private:
  ros::NodeHandle nh_;
  std::vector<std::vector<double>> waypoints_robot_a_;
  std::vector<std::vector<double>> waypoints_robot_b_;

  std::vector<double> waypoint_pose_a_;
  std::vector<double> waypoint_pose_b_;

  tesseract_environment::Command::Ptr addBox(double box_x, double box_y, double box_side);
};

}  // namespace tesseract_ros_examples



#endif /* SDU_COLLISION_FREE_DEMO_UR10E_COLLISION_FREE_DEMO_INCLUDE_UR10E_COLLISION_FREE_DEMO_UR10E_COLLISION_FREE_DEMO_H_ */
