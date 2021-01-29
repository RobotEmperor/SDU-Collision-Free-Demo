/*
 * ur10e_collision_free_demo.cpp
 *
 *  Created on: Nov 23, 2020
 *      Author: yik
 */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <jsoncpp/json/json.h>
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <ur10e_collision_free_demo/ur10e_collision_free_demo.h>

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

using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;
using namespace tesseract_visualization;
using namespace rw::math;

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";

/** @brief RViz Example Namespace */
const std::string EXAMPLE_MONITOR_NAMESPACE = "tesseract_ros_examples";

const double OFFSET = 0.005;

const std::string LINK_BOX_NAME = "box";
const std::string LINK_END_EFFECTOR_NAME = "iiwa_link_ee";

namespace online_planning_test
{
PickAndPlaceExample::PickAndPlaceExample(const ros::NodeHandle& nh, bool plotting, bool rviz)
                  : Example(plotting, rviz), nh_(nh)
                    {
  // total waypoints
  waypoints_robot_a_.clear();
  waypoints_robot_b_.clear();

  waypoint_pose_a_.resize(6);
  waypoint_pose_b_.resize(6);
                    }

Command::Ptr PickAndPlaceExample::addBox(double box_x, double box_y, double box_side)
{
  auto link_box = std::make_shared<Link>(LINK_BOX_NAME);

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->geometry = std::make_shared<tesseract_geometry::Box>(box_side, box_side, box_side);
  link_box->visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_box->collision.push_back(collision);

  auto joint_box = std::make_shared<Joint>("joint_box");
  joint_box->parent_link_name = "a_base_link";
  joint_box->child_link_name = LINK_BOX_NAME;
  joint_box->type = JointType::FIXED;
  joint_box->parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
  joint_box->parent_to_joint_origin_transform.translation() += Eigen::Vector3d(box_x, box_y, (box_side / 2.0) + OFFSET);

  return std::make_shared<tesseract_environment::AddCommand>(link_box, joint_box);
}
void PickAndPlaceExample::make_circle_waypoints(int direction_, double radious_)
{
  double x_,y_;
  double rotation_z_;
  double theta_;
  double offset_;
  x_ = 0;
  y_ = 0;
  rotation_z_ = 0;
  theta_ = 90*DEGREE2RADIAN;
  offset_ = 90*DEGREE2RADIAN;

  rw::math::Transform3D<> tf_small_pulley_to_circle_points_;
  rw::math::Transform3D<> tf_big_pulley_to_circle_points_;
  rw::math::Transform3D<> tf_big_pulley_to_small_pulley_;

  tf_big_pulley_to_small_pulley_ =  Transform3D<> (Vector3D<>(-0.130, 0, -0.005), EAA<>(0,0,0).toRotation3D()); // EAA


  // output is relative to pulley's center point.
  if(direction_ > 0)
  {
    for(int num = 1; num <= 2; num ++)
    {
      x_ = -radious_*sin(theta_*num + offset_);
      y_ = radious_*cos(theta_*num + offset_);
      rotation_z_ = 0*DEGREE2RADIAN*num;

      tf_small_pulley_to_circle_points_ = Transform3D<> (Vector3D<>(x_, y_, 0), RPY<>(rotation_z_,0,0).toRotation3D()); // RPY
      tf_big_pulley_to_circle_points_ = tf_big_pulley_to_small_pulley_*tf_small_pulley_to_circle_points_;

      waypoint_pose_a_[0] = tf_big_pulley_to_circle_points_.P()[0];
      waypoint_pose_a_[1] = tf_big_pulley_to_circle_points_.P()[1];
      waypoint_pose_a_[2] = tf_big_pulley_to_circle_points_.P()[2];
      waypoint_pose_a_[3] = RPY<> (tf_big_pulley_to_circle_points_.R())[0];
      waypoint_pose_a_[4] = RPY<> (tf_big_pulley_to_circle_points_.R())[1];
      waypoint_pose_a_[5] = RPY<> (tf_big_pulley_to_circle_points_.R())[2];

      waypoints_robot_a_.push_back(waypoint_pose_a_);
    }

  }
  else
  {
    theta_ =-15*DEGREE2RADIAN - offset_;
    for(int num = 1; num < 10; num ++)
    {
      x_ = radious_*sin(theta_*num - offset_);
      y_ = radious_*cos(theta_*num - offset_);
      rotation_z_ = -15*DEGREE2RADIAN*num - offset_ ;

      tf_big_pulley_to_circle_points_ = Transform3D<> (Vector3D<>(x_, y_, -0.02), RPY<>(rotation_z_,0,0).toRotation3D()); // RPY

      waypoint_pose_a_[0] = tf_big_pulley_to_circle_points_.P()[0];
      waypoint_pose_a_[1] = tf_big_pulley_to_circle_points_.P()[1];
      waypoint_pose_a_[2] = tf_big_pulley_to_circle_points_.P()[2];
      waypoint_pose_a_[3] = RPY<> (tf_big_pulley_to_circle_points_.R())[0];
      waypoint_pose_a_[4] = RPY<> (tf_big_pulley_to_circle_points_.R())[1];
      waypoint_pose_a_[5] = RPY<> (tf_big_pulley_to_circle_points_.R())[2];

      waypoints_robot_b_.push_back(waypoint_pose_a_);
    }
  }
}

bool PickAndPlaceExample::run()
{
  using tesseract_planning::CartesianWaypoint;
  using tesseract_planning::CompositeInstruction;
  using tesseract_planning::CompositeInstructionOrder;
  using tesseract_planning::Instruction;
  using tesseract_planning::ManipulatorInfo;
  using tesseract_planning::MoveInstruction;
  using tesseract_planning::PlanInstruction;
  using tesseract_planning::PlanInstructionType;
  using tesseract_planning::ProcessPlanningFuture;
  using tesseract_planning::ProcessPlanningRequest;
  using tesseract_planning::ProcessPlanningServer;
  using tesseract_planning::StateWaypoint;
  using tesseract_planning::Waypoint;
  using tesseract_planning_server::ROSProcessEnvironmentCache;

  /////////////////////////
  /// motion way points ///
  /////////////////////////

  //Robot A
  //in relative to big pulley
  // close to the pulley
  waypoint_pose_a_[0] = -0.130;
  waypoint_pose_a_[1] = 0;
  waypoint_pose_a_[2] = -0.02;
  waypoint_pose_a_[3] = -10*DEGREE2RADIAN;
  waypoint_pose_a_[4] = 0;
  waypoint_pose_a_[5] = 0;

  waypoints_robot_a_.push_back(waypoint_pose_a_);

  // insert belt to pulley groove

  waypoint_pose_a_[0] = -0.160;
  waypoint_pose_a_[1] = 0.015;
  waypoint_pose_a_[2] = -0.01;
  waypoint_pose_a_[3] = -10*DEGREE2RADIAN;
  waypoint_pose_a_[4] = 0;
  waypoint_pose_a_[5] = 0;

  waypoints_robot_a_.push_back(waypoint_pose_a_);

  double safty_margin_;
  safty_margin_ = 0.0007;
  make_circle_waypoints(1, 0.016 + safty_margin_);

  // std::cout << waypoints_robot_a_.size() << std::endl;
  /*
  for(int num = 0; num < waypoints_robot_a_.size(); num ++)
  {
    std::cout << "NUM: " <<num << "Values 0 : "<< waypoints_robot_a_[num][0] << std::endl;
    std::cout << "NUM: " <<num << "Values 1 : "<< waypoints_robot_a_[num][1] << std::endl;
    std::cout << "NUM: " <<num << "Values 2 : "<< waypoints_robot_a_[num][2] << std::endl;
    std::cout << "NUM: " <<num << "Values 3 : "<< waypoints_robot_a_[num][3] << std::endl;
    std::cout << "NUM: " <<num << "Values 4 : "<< waypoints_robot_a_[num][4] << std::endl;
    std::cout << "NUM: " <<num << "Values 5 : "<< waypoints_robot_a_[num][5] << std::endl;
  }
   */
  //tf_big_pulley_to_small_pulley_


  //Robot B


  /////////////
  /// SETUP ///
  /////////////

  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  // Pull ROS params
  std::string urdf_xml_string, srdf_xml_string;
  //  double box_side, box_x, box_y;
  nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);
  //  nh_.getParam("box_side", box_side);
  //  nh_.getParam("box_x", box_x);
  //  nh_.getParam("box_y", box_y);

  // Initialize the environment
  ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env_->init<OFKTStateSolver>(urdf_xml_string, srdf_xml_string, locator))
    return false;

  // Create monitor
  monitor_ = std::make_shared<tesseract_monitoring::EnvironmentMonitor>(env_, EXAMPLE_MONITOR_NAMESPACE);
  if (rviz_)
    monitor_->startPublishingEnvironment(tesseract_monitoring::EnvironmentMonitor::UPDATE_ENVIRONMENT);

  // Set default contact distance
  Command::Ptr cmd_default_dist = std::make_shared<tesseract_environment::ChangeDefaultContactMarginCommand>(0.005);
  if (!monitor_->applyCommand(*cmd_default_dist))
    return false;

  // Create plotting tool
  ROSPlottingPtr plotter = std::make_shared<tesseract_rosutils::ROSPlotting>(env_->getSceneGraph()->getRoot());
  if (rviz_)
    plotter->waitForConnection();

  // Set the robot initial state
  std::vector<std::string> joint_names;
  joint_names.push_back("a_shoulder_pan_joint");
  joint_names.push_back("a_shoulder_lift_joint");
  joint_names.push_back("a_elbow_joint");
  joint_names.push_back("a_wrist_1_joint");
  joint_names.push_back("a_wrist_2_joint");
  joint_names.push_back("a_wrist_3_joint");
  //joint_names.push_back("a_robotiq_85_right_inner_knuckle_joint");
  //joint_names.push_back("a_robotiq_85_right_finger_tip_joint");

  Eigen::VectorXd joint_pos(6);
  joint_pos(0) = 2.97326;
  joint_pos(1) = -1.6538;
  joint_pos(2) = -2.33488;
  joint_pos(3) = -2.28384;
  joint_pos(4) = -2.53001;
  joint_pos(5) = -3.13221;
  //joint_pos(6) = 0;
  //joint_pos(7) = 0;


  env_->setState(joint_names, joint_pos);

  // Add simulated box to environment
  //  Command::Ptr cmd = addBox(box_x, box_y, box_side);
  //  if (!monitor_->applyCommand(*cmd))
  //    return false;

  ////////////
  /// PICK ///
  ////////////
  if (rviz_)
    plotter->waitForInput();

  // Create Program
  CompositeInstruction pick_program("DEFAULT", CompositeInstructionOrder::ORDERED, ManipulatorInfo("ur10e_a"));

  Waypoint pick_swp = StateWaypoint(joint_names, joint_pos);
  PlanInstruction start_instruction(pick_swp, PlanInstructionType::START);
  pick_program.setStartInstruction(start_instruction);

  // Define the final pose (on top of the box)
  //  Eigen::Isometry3d pick_final_pose;
  //  pick_final_pose.linear() = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0).matrix();
  //  pick_final_pose.translation() = Eigen::Vector3d(0.639757210413974, 0.07993900394775277, 0.2449995438351456 + OFFSET);  // rviz world
  //  Waypoint pick_wp1 = CartesianWaypoint(pick_final_pose);
  //
  //  // Define the approach pose
  //  Eigen::Isometry3d pick_approach_pose = pick_final_pose;
  //  pick_approach_pose.translation() += Eigen::Vector3d(0.0, 0.0, 0.15);
  //  Waypoint pick_wp0 = CartesianWaypoint(pick_approach_pose);
  //
  //  // Plan freespace from start
  //  PlanInstruction pick_plan_a0(pick_wp0, PlanInstructionType::FREESPACE, "FREESPACE");
  //  pick_plan_a0.setDescription("From start to pick Approach");
  //
  //  // Plan cartesian approach
  //  PlanInstruction pick_plan_a1(pick_wp1, PlanInstructionType::LINEAR, "CARTESIAN");
  //  pick_plan_a1.setDescription("Pick Approach");

  //path planning
  // Define the approach pose
  Eigen::Isometry3d temp_pose;
  rw::math::Transform3D<> tf_world_to_a_base_link_;
  rw::math::Transform3D<> tf_a_base_link_to_big_pulley_;
  rw::math::Transform3D<> tf_big_pulley_to_waypoints_;
  rw::math::Transform3D<> tf_world_to_waypoints_;

  tf_world_to_a_base_link_ = Transform3D<> (Vector3D<>(0, 0, 0.05), RPY<> (180*DEGREE2RADIAN,0,0).toRotation3D()); // RPY
  tf_a_base_link_to_big_pulley_ = Transform3D<> (Vector3D<>(-0.739757210413974, 0.07993900394775277, 0.2449995438351456), EAA<>(-0.7217029684216122, -1.7591780460014375, 1.7685571865188172).toRotation3D()); // RPY

  for(int num = 0; num < waypoints_robot_a_.size(); num ++)
  {

    tf_big_pulley_to_waypoints_ = Transform3D<> (Vector3D<>(waypoints_robot_a_[num][0], waypoints_robot_a_[num][1], waypoints_robot_a_[num][2]), RPY<>(waypoints_robot_a_[num][3],waypoints_robot_a_[num][4],waypoints_robot_a_[num][5]).toRotation3D()); // RPY
    tf_world_to_waypoints_ = tf_world_to_a_base_link_ * tf_a_base_link_to_big_pulley_*tf_big_pulley_to_waypoints_;


    temp_pose.linear() = Eigen::Quaterniond(Quaternion<> (tf_world_to_waypoints_.R())[3],Quaternion<> (tf_world_to_waypoints_.R())[0],Quaternion<> (tf_world_to_waypoints_.R())[1], Quaternion<> (tf_world_to_waypoints_.R())[2]).matrix();
    temp_pose.translation() = Eigen::Vector3d(tf_world_to_waypoints_.P()[0], tf_world_to_waypoints_.P()[1], tf_world_to_waypoints_.P()[2]);  // rviz world

    Waypoint temp_wp = CartesianWaypoint(temp_pose);

    PlanInstruction pick_plan_a1(temp_wp, PlanInstructionType::FREESPACE, "DEFAULT");
    pick_plan_a1.setDescription("pose_1");


    pick_program.push_back(pick_plan_a1);
  }

  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ROSProcessEnvironmentCache>(monitor_), 5);
  planning_server.loadDefaultProcessPlanners();


  // Create TrajOpt Profile
  auto trajopt_plan_profile = std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();
  trajopt_plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 10);

  auto trajopt_composite_profile = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();
  trajopt_composite_profile->collision_constraint_config.enabled = false;
  trajopt_composite_profile->collision_cost_config.safety_margin = 0.0005;
  trajopt_composite_profile->collision_cost_config.coeff = 50;


  auto trajopt_solver_profile = std::make_shared<tesseract_planning::TrajOptDefaultSolverProfile>();
  trajopt_solver_profile->opt_info.max_iter = 100;

  // Add profile to Dictionary
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptPlanProfile>("CARTESIAN", trajopt_plan_profile);
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptCompositeProfile>("DEFAULT",
      trajopt_composite_profile);
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptSolverProfile>("DEFAULT",
      trajopt_solver_profile);

  ROS_INFO("Pick plan");

  // Create Process Planning Request
  ProcessPlanningRequest pick_request;
  pick_request.name = tesseract_planning::process_planner_names::TRAJOPT_PLANNER_NAME;
  pick_request.instructions = Instruction(pick_program);

  // Print Diagnostics
  pick_request.instructions.print("Program: ");


  //for(int num = 0 ; num < 24 ; num++)
  //std::cout << "!!!!!!!" << joint_names.size() << std::endl;

  // Solve process plan
  ProcessPlanningFuture pick_response = planning_server.run(pick_request);
  planning_server.waitForAll();

  // Plot Process Trajectory
  if (rviz_ && plotter != nullptr && plotter->isConnected())
  {
    plotter->waitForInput();
    const auto* cp = pick_response.results->cast_const<CompositeInstruction>();
    tesseract_common::Toolpath toolpath = tesseract_planning::toToolpath(*cp, env_);
    tesseract_common::JointTrajectory trajectory = tesseract_planning::toJointTrajectory(*cp);
    plotter->plotMarker(ToolpathMarker(toolpath));
    plotter->plotTrajectory(trajectory, env_->getStateSolver());
  }

  //  /////////////
  //  /// PLACE ///
  //  /////////////
  //
  //  if (rviz_)
  //    plotter->waitForInput();
  //
  //  // Detach the simulated box from the world and attach to the end effector
  //  tesseract_environment::Commands cmds;
  //  auto joint_box2 = std::make_shared<Joint>("joint_box2");
  //  joint_box2->parent_link_name = LINK_END_EFFECTOR_NAME;
  //  joint_box2->child_link_name = LINK_BOX_NAME;
  //  joint_box2->type = JointType::FIXED;
  //  joint_box2->parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
  //  joint_box2->parent_to_joint_origin_transform.translation() += Eigen::Vector3d(0, 0, box_side / 2.0);
  //  cmds.push_back(std::make_shared<tesseract_environment::MoveLinkCommand>(joint_box2));
  //  cmds.push_back(
  //      std::make_shared<tesseract_environment::AddAllowedCollisionCommand>(LINK_BOX_NAME, "iiwa_link_ee", "Never"));
  //  cmds.push_back(
  //      std::make_shared<tesseract_environment::AddAllowedCollisionCommand>(LINK_BOX_NAME, "iiwa_link_7", "Never"));
  //  cmds.push_back(
  //      std::make_shared<tesseract_environment::AddAllowedCollisionCommand>(LINK_BOX_NAME, "iiwa_link_6", "Never"));
  //  monitor_->applyCommands(cmds);
  //
  //  // Get the last move instruction
  //  const CompositeInstruction* pick_composite = pick_response.results->cast_const<CompositeInstruction>();
  //  const MoveInstruction* pick_final_state = tesseract_planning::getLastMoveInstruction(*pick_composite);
  //
  //  // Retreat to the approach pose
  //  Eigen::Isometry3d retreat_pose = pick_approach_pose;
  //
  //  // Define some place locations.
  //  Eigen::Isometry3d bottom_right_shelf, bottom_left_shelf, middle_right_shelf, middle_left_shelf, top_right_shelf,
  //      top_left_shelf;
  //  bottom_right_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  //  bottom_right_shelf.translation() = Eigen::Vector3d(0.148856, 0.73085, 0.906);
  //  bottom_left_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  //  bottom_left_shelf.translation() = Eigen::Vector3d(-0.148856, 0.73085, 0.906);
  //  middle_right_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  //  middle_right_shelf.translation() = Eigen::Vector3d(0.148856, 0.73085, 1.16);
  //  middle_left_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  //  middle_left_shelf.translation() = Eigen::Vector3d(-0.148856, 0.73085, 1.16);
  //  top_right_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  //  top_right_shelf.translation() = Eigen::Vector3d(0.148856, 0.73085, 1.414);
  //  top_left_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  //  top_left_shelf.translation() = Eigen::Vector3d(-0.148856, 0.73085, 1.414);
  //
  //  // Set the target pose to middle_left_shelf
  //  Eigen::Isometry3d place_pose = middle_left_shelf;
  //
  //  // Setup approach for place move
  //  Eigen::Isometry3d place_approach_pose = place_pose;
  //  place_approach_pose.translation() += Eigen::Vector3d(0.0, -0.25, 0);
  //
  //  // Create Program
  //  CompositeInstruction place_program("DEFAULT", CompositeInstructionOrder::ORDERED, ManipulatorInfo("Manipulator"));
  //
  //  PlanInstruction place_start_instruction(pick_final_state->getWaypoint(), PlanInstructionType::START);
  //  place_program.setStartInstruction(place_start_instruction);
  //
  //  // Define the approach pose
  //  Waypoint place_wp0 = CartesianWaypoint(retreat_pose);
  //
  //  // Define the final pose approach
  //  Waypoint place_wp1 = CartesianWaypoint(place_approach_pose);
  //
  //  // Define the final pose
  //  Waypoint place_wp2 = CartesianWaypoint(place_pose);
  //
  //  // Plan cartesian retraction from picking up the box
  //  PlanInstruction place_plan_a0(place_wp0, PlanInstructionType::LINEAR, "CARTESIAN");
  //  place_plan_a0.setDescription("Place retraction");
  //
  //  // Plan freespace to approach for box drop off
  //  PlanInstruction place_plan_a1(place_wp1, PlanInstructionType::FREESPACE, "FREESPACE");
  //  place_plan_a1.setDescription("Place Freespace");
  //
  //  // Plan cartesian approach to box drop location
  //  PlanInstruction place_plan_a2(place_wp2, PlanInstructionType::LINEAR, "CARTESIAN");
  //  place_plan_a2.setDescription("Place approach");
  //
  //  // Add Instructions to program
  //  place_program.push_back(place_plan_a0);
  //  place_program.push_back(place_plan_a1);
  //  place_program.push_back(place_plan_a2);
  //
  //  // Create Process Planning Request
  //  ProcessPlanningRequest place_request;
  //  place_request.name = tesseract_planning::process_planner_names::TRAJOPT_PLANNER_NAME;
  //  place_request.instructions = Instruction(place_program);
  //
  //  // Print Diagnostics
  //  place_request.instructions.print("Program: ");
  //
  //  // Solve process plan
  //  ProcessPlanningFuture place_response = planning_server.run(place_request);
  //  planning_server.waitForAll();

  // Plot Process Trajectory
  //  if (rviz_ && plotter != nullptr && plotter->isConnected())
  //  {
  //    plotter->waitForInput();
  //    const auto* ci = place_response.results->cast_const<tesseract_planning::CompositeInstruction>();
  //    tesseract_common::Toolpath toolpath = tesseract_planning::toToolpath(*ci, env_);
  //    tesseract_common::JointTrajectory trajectory = tesseract_planning::toJointTrajectory(*ci);
  //    plotter->plotMarker(ToolpathMarker(toolpath));
  //    plotter->plotTrajectory(trajectory, env_->getStateSolver());
  //  }

  if (rviz_)
    plotter->waitForInput();

  ROS_INFO("Done");
  return true;
}
}  // namespace tesseract_ros_examples

