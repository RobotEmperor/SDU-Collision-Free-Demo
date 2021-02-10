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
#include <ur10e_collision_free_demo/force_constraint.h>
#include <rw/math.hpp>

#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_command_language/utils/get_instruction_utils.h>
#include <tesseract_process_managers/taskflow_generators/trajopt_taskflow.h>
#include <tesseract_planning_server/tesseract_planning_server.h>
#include <tesseract_visualization/markers/toolpath_marker.h>


#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
#include <boost/filesystem/path.hpp>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_scene_graph/utils.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/kinematic_terms.hpp>
#include <trajopt/common.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_utils/clock.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

#include <trajopt/problem_description.hpp>

#define DEGREE2RADIAN M_PI/180.0
#define RADIAN2DEGREE  180.0/M_PI

using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;
using namespace tesseract_visualization;
using namespace rw::math;
using namespace util;

namespace online_planning_test
{
/**
 * @brief An example of a robot picking up a box and placing it on a shelf leveraging
 * tesseract and trajopt to generate the motion trajectory.
 */
class TestExample : public Example
{
  public:
    TestExample(const ros::NodeHandle& nh, bool plotting, bool rviz);
    ~TestExample() override = default;
    TestExample(const TestExample&) = default;
    TestExample& operator=(const TestExample&) = default;
    TestExample(TestExample&&) = default;
    TestExample& operator=(TestExample&&) = default;

    void make_circle_waypoints(int direction_, double radious_);
    bool run() override;

  private:
    ros::NodeHandle nh_;
    std::vector<std::vector<double>> waypoints_robot_a_;
    std::vector<std::vector<double>> waypoints_robot_b_;

    std::vector<double> waypoint_pose_a_;
    std::vector<double> waypoint_pose_b_;

};

}  // namespace tesseract_ros_examples



#endif /* SDU_COLLISION_FREE_DEMO_UR10E_COLLISION_FREE_DEMO_INCLUDE_UR10E_COLLISION_FREE_DEMO_UR10E_COLLISION_FREE_DEMO_H_ */
