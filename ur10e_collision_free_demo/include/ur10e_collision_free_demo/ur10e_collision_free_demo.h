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
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/SetBool.h>
#include <ifopt/problem.h>

#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
#include <tesseract_rosutils/plotting.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <ur10e_collision_free_demo/example.h>

namespace ur10e_collision_free_demo
{
/**
 * @brief This example demonstrates running the inner loop of TrajOpt in an "online" manner. As the environment changes,
 * TrajOpt dynamically adjusts to avoid collisions and follow the given toolpath
 */
class OnlinePlanningExample : public tesseract_ros_examples::Example
{
public:
  OnlinePlanningExample(const ros::NodeHandle& nh, bool plotting, bool rviz, int steps, double box_size);

  ~OnlinePlanningExample() override = default;
  OnlinePlanningExample(const OnlinePlanningExample&) = default;
  OnlinePlanningExample& operator=(const OnlinePlanningExample&) = default;
  OnlinePlanningExample(OnlinePlanningExample&&) = default;
  OnlinePlanningExample& operator=(OnlinePlanningExample&&) = default;

  /**
   * @brief Runs the example
   * @return True if successful
   */
  bool run() override;

  /**
   * @brief Sets up the IFOPT problem
   * @return True if successful
   */
  bool setupProblem();

  /**
   * @brief Runs the online planning process, occoasionally checking ROS callbacks until realtime_running_ = false
   * @return True if successful
   */
  bool onlinePlan();

  /** @brief ROS subscriber callback used to update the dynamic collision obstacle and target position */
  void subscriberCallback(const sensor_msgs::JointState::ConstPtr& joint_state);

  /** @brief ROS service callback for toggling the online planning on/off */
  bool toggleRealtime(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

private:
  ros::NodeHandle nh_;
  int steps_;
  double box_size_;

  tesseract_kinematics::ForwardKinematics::Ptr manipulator_fk_;
  tesseract_kinematics::InverseKinematics::Ptr manipulator_ik_;
  std::shared_ptr<tesseract_rosutils::ROSPlotting> plotter_;
  tesseract_environment::AdjacencyMap::Ptr manipulator_adjacency_map_;

  trajopt::TrajArray current_trajectory_;
  Eigen::Isometry3d target_pose_delta_;
  Eigen::Isometry3d target_pose_base_frame_;
  // We need to keep this around so we can update it
  trajopt::CartPosConstraint::Ptr target_pose_constraint_;

  std::vector<std::string> joint_names_;
  ros::Subscriber joint_state_subscriber_;
  ros::ServiceServer toggle_realtime_service_;
  ifopt::Problem nlp_;

  bool realtime_running_;
};
}  // namespace tesseract_ros_examples



#endif /* SDU_COLLISION_FREE_DEMO_UR10E_COLLISION_FREE_DEMO_INCLUDE_UR10E_COLLISION_FREE_DEMO_UR10E_COLLISION_FREE_DEMO_H_ */
